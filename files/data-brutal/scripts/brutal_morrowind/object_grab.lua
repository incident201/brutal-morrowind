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
local dualGrip = false
local savedStance = nil
local stateTime = 0
local useDown = false
local throwQueued = false
local releaseFired = false
local throwAdrenaline = false
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
    heldItem, requestedItem = nil, nil
    state, stateTime = 'idle', 0
    throwQueued, releaseFired, throwAdrenaline = false, false, false
end

local function aimVectors()
    local eyePos = camera.getPosition()
    local yaw = camera.getYaw()
    local pitch = camera.getPitch()
    local cp = math.cos(pitch)
    local forward = util.vector3(math.sin(yaw) * cp, math.cos(yaw) * cp, -math.sin(pitch))
    return eyePos, forward:normalize()
end

local function canGrab(obj)
    if not obj or not obj:isValid() or not obj.enabled or obj.cell ~= self.cell then return false end
    return type(types.Actor.canHoldObject) == 'function' and types.Actor.canHoldObject(obj)
end

local function findTarget()
    local eyePos, forward = aimVectors()
    local far = eyePos + forward * PICKUP_REACH
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
    if I.UI.getMode() or types.Actor.isDead(self) or not types.Actor.canMove(self)
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

local function fireRelease(throwSpeed, adrenaline)
    local eyePos, forward = aimVectors()
    core.sendGlobalEvent('BrutalGrabRelease', {
        player = self,
        eyePos = eyePos,
        throwDir = forward,
        throwSpeed = throwSpeed,
        adrenaline = adrenaline == true,
    })
end

local function startDrop(immediate)
    if state == 'idle' or state == 'dropping' then return false end
    if state == 'throwing' and releaseFired then
        if immediate then restoreStance() end
        return false
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
    if state ~= 'holding' and state ~= 'raising' and state ~= 'picking' then return false end
    local adren = I.BrutalAdrenaline and I.BrutalAdrenaline.spendThrow and I.BrutalAdrenaline.spendThrow() == true or false
    throwAdrenaline = adren
    releaseFired = false
    throwQueued = false
    state, stateTime = 'throwing', 0
    stats.throws = stats.throws + 1
    if adren then stats.adrenalineThrows = stats.adrenalineThrows + 1 end
    playSection(throwGroup(), 'start', 'stop', 0, false)
    return true
end

local function startGrab(explicitTarget)
    if state ~= 'idle' or not canAct() then return false end
    local target = explicitTarget or findTarget()
    if not canGrab(target) then
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
    core.sendGlobalEvent('BrutalGrabRequest', { player = self, object = target })
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
    if state == 'picking' or state == 'holding' then
        state, stateTime = 'raising', 0
        throwQueued = false
        playSection(grabGroup(), 'raise start', 'raise stop', 0, false)
        return true
    end
    return false
end

local function queueThrowRelease()
    if state == 'raising' then
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
        if (state == 'picking' or state == 'holding') and canAct() then
            beginThrowWindup()
        end
    else
        if state == 'raising' then
            queueThrowRelease()
        end
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

    if state == 'throwing' then
        local tg = throwGroup()
        local relT = textKeyDuration(tg, 'start', 'release', dualGrip and 0.2667 or 0.1333)
        local recT = textKeyDuration(tg, 'start', 'recover', dualGrip and 0.96 or 0.72)
        if not releaseFired then
            local eyePos, forward = aimVectors()
            local u = math.min(1, stateTime / math.max(0.04, relT))
            core.sendGlobalEvent('BrutalGrabUpdateView', {
                player = self, eyePos = eyePos, aimDir = forward,
                pullback = 7 - 15 * u, raiseOffset = 4 * (1 - u), dualGrip = dualGrip,
            })
        end
        if not releaseFired and stateTime >= relT then
            releaseFired = true
            self:sendEvent('BrutalObjectGrabSound', { kind = 'throw', adrenaline = throwAdrenaline })
            fireRelease(dualGrip and 920 or 1060, throwAdrenaline)
            heldItem = nil
            if (not useDmAnims() or settings.get('firstPersonAnimations') ~= 'darkMessiah')
                and I.BrutalCameraMotion and I.BrutalCameraMotion.pulse then
                I.BrutalCameraMotion.pulse(throwAdrenaline and 0.20 or 0.14, 0.07, 0, 0.28)
            end
        end
        if stateTime >= recT then restoreStance() end
        return
    end

    -- Picking, holding, or raising: safety checks and hold target tracking.
    if not canAct() then
        startDrop(true)
        return
    end
    local target = heldItem or requestedItem
    if not target or not target:isValid() or not target.enabled or target.cell ~= self.cell then
        startDrop(true)
        return
    end

    local pullback, raiseOffset = 0, 0
    if state == 'picking' then
        local pickDur = textKeyDuration(grabGroup(), 'pickup start', 'pickup stop', dualGrip and 0.30 or 0.25)
        local u = math.min(1, stateTime / math.max(0.05, pickDur))
        raiseOffset = -4 * (1 - u)
        if heldItem and stateTime >= pickDur then
            state, stateTime = 'holding', 0
            playSection(grabGroup(), 'hold start', 'hold stop', 100000, false)
        elseif not heldItem and stateTime > 0.6 then
            -- Hold request timed out / rejected cleanly.
            restoreStance()
            return
        end
    elseif state == 'raising' then
        local raiseDur = textKeyDuration(grabGroup(), 'raise start', 'raise stop', dualGrip and 0.24 or 0.20)
        local u = math.min(1, stateTime / math.max(0.04, raiseDur))
        pullback = 7 * u
        raiseOffset = 4 * u
        if throwQueued and stateTime >= raiseDur then
            startThrow()
            return
        end
    end

    if heldItem then
        local eyePos, forward = aimVectors()
        core.sendGlobalEvent('BrutalGrabUpdateView', {
            player = self, eyePos = eyePos, aimDir = forward,
            pullback = pullback, raiseOffset = raiseOffset, dualGrip = dualGrip,
        })
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
            dualGrip = data.dualGrip == true
            stateTime = 0
            stats.grabs = stats.grabs + 1
            self:sendEvent('BrutalObjectGrabSound', { kind = 'pickup', dualGrip = dualGrip })
            if I.BrutalCameraMotion and I.BrutalCameraMotion.pulse then
                I.BrutalCameraMotion.pulse(0.04, -0.025, 1.5, 0.20)
            end
            playSection(grabGroup(), 'pickup start', 'pickup stop', 0, false)
        end,
        BrutalObjectHoldFailed = function()
            if state == 'picking' then
                stats.rejected = stats.rejected + 1
                restoreStance()
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
        onLoad = function()
            heldItem, requestedItem, dualGrip, savedStance = nil, nil, false, nil
            state, stateTime, useDown, throwQueued, releaseFired, throwAdrenaline = 'idle', 0, false, false, false, false
            cancelAnims()
            if types.Actor.setBrutalCarryingObject then
                types.Actor.setBrutalCarryingObject(self, false)
            end
        end,
    },
}
