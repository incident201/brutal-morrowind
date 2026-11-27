-- First-person Dark Messiah object grab, hold, drop (G), and throw (LMB).
local self = require('openmw.self')
local core = require('openmw.core')
local types = require('openmw.types')
local util = require('openmw.util')
local input = require('openmw.input')
local camera = require('openmw.camera')
local nearby = require('openmw.nearby')
local animation = require('openmw.animation')
local async = require('openmw.async')
local I = require('openmw.interfaces')
local settings = require('scripts.brutal_morrowind.settings')
local bindings = require('scripts.brutal_morrowind.input_bindings')

local GRAB_SOLO, GRAB_DUAL = 'brutal_grab_solo', 'brutal_grab_dual'
local THROW_SOLO, THROW_DUAL = 'brutal_throw_solo', 'brutal_throw_dual'
local PICKUP_REACH = 155

local state = 'idle'
local heldItem, requestedItem = nil, nil
local heldFragment = nil
local dualGrip = false
local savedStance = nil
local stateTime = 0
local useDown = false
local pickupCuePlayed = false
local throwAfterPickup = false
local throwQueued = false
local releaseFired = false
local throwAdrenaline = false
local throwSerial, throwId, throwRequested = 0, nil, false
local stats = {
    grabs = 0, drops = 0, throws = 0, adrenalineThrows = 0,
    headHits = 0, bodyHits = 0, knockdowns = 0, rejected = 0,
    lastReaction = '', lastHit = nil,
}

local function grabGroup() return dualGrip and GRAB_DUAL or GRAB_SOLO end
local function throwGroup() return dualGrip and THROW_DUAL or THROW_SOLO end

local function useDmAnims()
    return settings.get('firstPersonAnimations') == 'darkMessiah'
        and animation.hasGroup(self, grabGroup())
        and animation.hasGroup(self, throwGroup())
end

local function textKeyDuration(group, startKey, stopKey, fallback)
    local s = animation.getTextKeyTime(self, group .. ': ' .. startKey)
    local e = animation.getTextKeyTime(self, group .. ': ' .. stopKey)
    if s and e and e > s then return e - s end
    return fallback
end

local function playSection(group, startKey, stopKey, loops, autoDisable)
    if not useDmAnims() then return end
    local other = group == grabGroup() and throwGroup() or grabGroup()
    animation.cancel(self, other)
    animation.cancel(self, group)
    I.AnimationController.playBlendedAnimation(group, {
        priority = animation.PRIORITY.Scripted,
        blendMask = animation.BLEND_MASK.UpperBody,
        startKey = startKey,
        stopKey = stopKey,
        startPoint = 0,
        speed = 1,
        autoDisable = autoDisable == true,
        loops = loops or 0,
        forceLoop = (loops or 0) > 0,
    })
end

local function cancelAnims()
    for _, g in ipairs({ GRAB_SOLO, GRAB_DUAL, THROW_SOLO, THROW_DUAL }) do
        animation.cancel(self, g)
    end
end

local function restoreStance()
    cancelAnims()
    if types.Actor.setBrutalCarryingObject then
        types.Actor.setBrutalCarryingObject(self, false)
    end
    local prev = savedStance
    savedStance = nil
    if prev and prev ~= types.Actor.STANCE.Nothing and not types.Actor.isDead(self) then
        types.Actor.setStance(self, prev)
    end
    heldItem, requestedItem, heldFragment = nil, nil, nil
    state, stateTime = 'idle', 0
    pickupCuePlayed = false
    throwAfterPickup = false
    throwQueued, releaseFired, throwAdrenaline = false, false, false
    throwId, throwRequested = nil, false
end

local function aimVectors()
    local viewToWorld = camera.getViewTransform():inverse()
    local eyePos = viewToWorld:apply(util.vector3(0, 0, 0))
    local forward = viewToWorld:apply(util.vector3(0, 0, -32)) - eyePos
    return eyePos, forward:normalize()
end

local function canGrab(obj)
    if not obj or not obj:isValid() or not obj.enabled or not obj.cell or not obj.cell:isInSameSpace(self) then return false end
    return type(types.Actor.canHoldObject) == 'function' and types.Actor.canHoldObject(obj)
end

local function findTarget()
    local eyePos, forward = aimVectors()
    local far = eyePos + forward * PICKUP_REACH
    if types.Actor.findDetachedGrabFragment then
        local corpse, fragment = types.Actor.findDetachedGrabFragment(self, eyePos, far)
        if corpse then return corpse, fragment end
    end
    local okRender, renderHit = pcall(nearby.castRenderingRay, eyePos, far, { ignore = self })
    if okRender and renderHit and renderHit.hit then
        if canGrab(renderHit.hitObject) then return renderHit.hitObject end
        far = renderHit.hitPos
    end
    local physMask = nearby.COLLISION_TYPE.World + nearby.COLLISION_TYPE.Door + nearby.COLLISION_TYPE.HeightMap
    local physHit = nearby.castRay(eyePos, far, { ignore = self, collisionType = physMask })
    if physHit and physHit.hit then
        if canGrab(physHit.hitObject) then return physHit.hitObject end
    end
    -- Fallback for slender/thin items near center crosshair whose NIF bounds don't fill center pixel.
    local best, bestScore = nil, 0.94
    for _, item in ipairs(nearby.items) do
        if canGrab(item) then
            local box = item:getBoundingBox()
            local toItem = (box and box.center or item.position) - eyePos
            local dist = toItem:length()
            if dist > 4 and dist <= PICKUP_REACH then
                local dir = toItem / dist
                local dot = dir:dot(forward)
                if dot >= bestScore then
                    local los = nearby.castRay(eyePos, eyePos + toItem * 0.96, { ignore = self, collisionType = physMask })
                    if not los.hit or los.hitObject == item then
                        best, bestScore = item, dot
                    end
                end
            end
        end
    end
    return best
end

local function canAct()
    if not settings.get('enabled') or not settings.get('objectGrabEnabled') then return false end
    local switches = types.Player.CONTROL_SWITCH
    if not types.Player.getControlSwitch(self, switches.Controls)
        or not types.Player.getControlSwitch(self, switches.Fighting)
        or I.UI.getMode() or types.Actor.isDead(self) or not types.Actor.canMove(self)
        or types.Actor.isSwimming(self) or camera.getMode() ~= camera.MODE.FirstPerson then
        return false
    end
    if I.BrutalSlide and I.BrutalSlide.status().phase ~= 0 then return false end
    if I.BrutalClinch then
        local c = I.BrutalClinch.status()
        if c.active or c.pending then return false end
    end
    return true
end

local function fireRelease(throwSpeed, adrenaline, id)
    local eyePos, forward = aimVectors()
    core.sendGlobalEvent('BrutalGrabRelease', {
        player = self,
        eyePos = eyePos,
        throwDir = forward,
        throwSpeed = throwSpeed,
        adrenaline = adrenaline == true,
        throwId = id,
    })
end

local function requestThrow()
    throwRequested = true
    fireRelease(dualGrip and 920 or 1060, throwAdrenaline, throwId)
end

local function startDrop(immediate)
    if state == 'idle' or state == 'dropping' then return false end
    if state == 'throwing' and releaseFired then
        if immediate then restoreStance() end
        return false
    end
    if state == 'throwing' and not throwRequested and throwAdrenaline then
        I.BrutalAdrenaline.cancelThrow(throwId)
    end
    fireRelease(0, false)
    heldItem, requestedItem = nil, nil
    stats.drops = stats.drops + 1
    if immediate then
        restoreStance()
        return true
    end
    self:sendEvent('BrutalObjectGrabSound', { kind = 'drop' })
    if I.BrutalCameraMotion and I.BrutalCameraMotion.pulse then
        I.BrutalCameraMotion.pulse(-0.035, 0.02, -1.5, 0.22)
    end
    state, stateTime = 'dropping', 0
    throwQueued = false
    playSection(grabGroup(), 'drop start', 'drop stop', 0, true)
    return true
end

local function startThrow()
    if not heldItem or (state ~= 'holding' and state ~= 'raising' and state ~= 'picking') or not canAct() then
        return false
    end
    throwSerial = throwSerial + 1
    throwId = throwSerial
    local adrenaline = I.BrutalAdrenaline
    throwAdrenaline = adrenaline and adrenaline.reserveThrow(throwId) == true or false
    throwRequested = false
    releaseFired = false
    throwQueued = false
    state, stateTime = 'throwing', 0
    local hasClip = useDmAnims()
    playSection(throwGroup(), 'start', 'stop', 0, false)
    if hasClip then requestThrow() end
    return true
end

local function startGrab(explicitTarget)
    if state ~= 'idle' or not canAct() then return false end
    if I.BrutalMorrowind then
        local melee = I.BrutalMorrowind.status()
        if melee.kicking or melee.bashing then return false end
    end
    local target, fragment = explicitTarget, nil
    if not target then target, fragment = findTarget() end
    if not fragment and not canGrab(target) then
        stats.rejected = stats.rejected + 1
        return false
    end
    savedStance = types.Actor.getStance(self)
    if types.Actor.setBrutalCarryingObject then
        types.Actor.setBrutalCarryingObject(self, true)
    end
    if savedStance ~= types.Actor.STANCE.Nothing then
        types.Actor.setStance(self, types.Actor.STANCE.Nothing)
    end
    requestedItem = target
    state, stateTime = 'picking', 0
    core.sendGlobalEvent('BrutalGrabRequest', { player = self, object = target, fragment = fragment })
    return true
end

local function onGrabTrigger()
    if state == 'idle' then
        startGrab()
    elseif state == 'picking' or state == 'holding' or state == 'raising' then
        startDrop(false)
    end
end

local function beginThrowWindup()
    if not canAct() then return false end
    if state == 'picking' then
        throwAfterPickup = true
        return true
    end
    if state == 'holding' then
        state, stateTime = 'raising', 0
        throwQueued = false
        playSection(grabGroup(), 'raise start', 'raise stop', 0, false)
        return true
    end
    return false
end

local function queueThrowRelease()
    if state == 'raising' or (state == 'picking' and throwAfterPickup) then
        throwQueued = true
        return true
    end
    return false
end

local function onUseChanged(down)
    down = down == true
    if down == useDown then return end
    useDown = down
    if down then
        if heldItem and (state == 'picking' or state == 'holding') then
            beginThrowWindup()
        end
    else
        queueThrowRelease()
    end
end

input.registerTriggerHandler(bindings.GRAB, async:callback(onGrabTrigger))
input.registerActionHandler('Use', async:callback(onUseChanged))

local function onUpdate(dt)
    if dt <= 0 or state == 'idle' then return end
    stateTime = stateTime + dt

    if state == 'dropping' then
        local dur = textKeyDuration(grabGroup(), 'drop start', 'drop stop', dualGrip and 0.25 or 0.22)
        if stateTime >= dur then restoreStance() end
        return
    end
    if not canAct() then
        startDrop(true)
        return
    end

    if state == 'throwing' then
        local tg = throwGroup()
        local relT = textKeyDuration(tg, 'start', 'release', dualGrip and 0.2667 or 0.1333)
        local recT = textKeyDuration(tg, 'start', 'recover', dualGrip and 0.96 or 0.72)
        if not throwRequested and stateTime >= relT then requestThrow() end
        if releaseFired and stateTime >= recT then restoreStance() end
        return
    end

    -- Picking, holding, or raising: the engine publishes the evaluated grip.
    local target = heldItem or requestedItem
    if not target or not target:isValid() or not target.enabled or not target.cell or not target.cell:isInSameSpace(self) then
        startDrop(true)
        return
    end

    if state == 'picking' then
        local pickDur = textKeyDuration(grabGroup(), 'pickup start', 'pickup stop', dualGrip and 0.30 or 0.25)
        local contact = useDmAnims() and textKeyDuration(grabGroup(), 'pickup start', 'pickup contact', 0) or 0
        if heldItem and not pickupCuePlayed and stateTime >= contact then
            pickupCuePlayed = true
            self:sendEvent('BrutalObjectGrabSound', { kind = 'pickup', dualGrip = dualGrip })
            if I.BrutalCameraMotion and I.BrutalCameraMotion.pulse then
                I.BrutalCameraMotion.pulse(0.04, -0.025, 1.5, 0.20)
            end
        end
        if heldItem and stateTime >= pickDur then
            state, stateTime = 'holding', 0
            if throwAfterPickup then
                local released = throwQueued
                throwAfterPickup = false
                beginThrowWindup()
                if released then queueThrowRelease() end
            else
                playSection(grabGroup(), 'hold start', 'hold stop', 100000, false)
            end
        elseif not heldItem and stateTime > 0.6 then
            -- Hold request timed out / rejected cleanly.
            restoreStance()
            return
        end
    elseif state == 'raising' then
        local raiseDur = textKeyDuration(grabGroup(), 'raise start', 'raise stop', dualGrip and 0.24 or 0.20)
        if throwQueued and stateTime >= raiseDur then
            startThrow()
            return
        end
    end


end

return {
    interfaceName = 'BrutalObjectGrab',
    interface = {
        version = 1,
        grab = function(explicitTarget) return startGrab(explicitTarget) end,
        drop = function() return startDrop(false) end,
        throwStart = function() return beginThrowWindup() end,
        throwRelease = function() return queueThrowRelease() end,
        throwNow = function() return startThrow() end,
        status = function()
            return {
                state = state,
                active = state ~= 'idle',
                heldObject = heldItem,
                heldFragment = heldFragment,
                dualGrip = dualGrip,
                savedStance = savedStance,
                grabs = stats.grabs,
                drops = stats.drops,
                throws = stats.throws,
                adrenalineThrows = stats.adrenalineThrows,
                headHits = stats.headHits,
                bodyHits = stats.bodyHits,
                knockdowns = stats.knockdowns,
                rejected = stats.rejected,
                lastReaction = stats.lastReaction,
                lastHit = stats.lastHit,
            }
        end,
    },
    eventHandlers = {
        BrutalObjectHeld = function(data)
            if state ~= 'picking' or not data or data.object ~= requestedItem then
                if data and data.object then fireRelease(0, false) end
                return
            end
            heldItem = data.object
            heldFragment = data.fragment ~= 0 and data.fragment or nil
            dualGrip = data.dualGrip == true
            stateTime = 0
            stats.grabs = stats.grabs + 1
            playSection(grabGroup(), 'pickup start', 'pickup stop', 0, false)
        end,
        BrutalObjectHoldFailed = function()
            if state == 'picking' then
                stats.rejected = stats.rejected + 1
                restoreStance()
            end
        end,
        BrutalObjectThrowResolved = function(data)
            if not data or data.throwId ~= throwId or state ~= 'throwing' or releaseFired then return end
            if not data.released then restoreStance();return end
            releaseFired = true
            heldItem, heldFragment = nil, nil
            stats.throws = stats.throws + 1
            if throwAdrenaline then stats.adrenalineThrows = stats.adrenalineThrows + 1 end
            self:sendEvent('BrutalObjectGrabSound', { kind = 'throw', adrenaline = throwAdrenaline })
            if not useDmAnims() and I.BrutalCameraMotion and I.BrutalCameraMotion.pulse then
                I.BrutalCameraMotion.pulse(throwAdrenaline and 0.20 or 0.14, 0.07, 0, 0.28)
            end
        end,
        BrutalThrownObjectHit = function(data)
            if type(data) ~= 'table' then return end
            if data.headHit then stats.headHits = stats.headHits + 1 else stats.bodyHits = stats.bodyHits + 1 end
            if data.knockedDown then stats.knockdowns = stats.knockdowns + 1 end
            stats.lastReaction = data.reaction or ''
            stats.lastHit = data
        end,
    },
    engineHandlers = {
        onUpdate = onUpdate,
        onSave = function() return {stance = savedStance, throwSerial = throwSerial} end,
        onLoad = function(data)
            local previousStance = data and data.stance
            heldItem, requestedItem, heldFragment, dualGrip, savedStance = nil, nil, nil, false, nil
            state, stateTime, useDown, throwQueued, releaseFired, throwAdrenaline = 'idle', 0, false, false, false, false
            pickupCuePlayed = false
            throwAfterPickup = false
            throwSerial, throwId, throwRequested = data and data.throwSerial or 0, nil, false
            cancelAnims()
            if types.Actor.setBrutalCarryingObject then
                types.Actor.setBrutalCarryingObject(self, false)
            end
            if (previousStance == types.Actor.STANCE.Weapon or previousStance == types.Actor.STANCE.Spell)
                and types.Actor.getStance(self) == types.Actor.STANCE.Nothing then
                types.Actor.setStance(self, previousStance)
            end
        end,
    },
}
