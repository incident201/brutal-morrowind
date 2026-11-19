local world = require('openmw.world')
local types = require('openmw.types')
local util = require('openmw.util')
local core = require('openmw.core')
local vfs = require('openmw.vfs')
local C = require('scripts.brutal_morrowind.config')
local S = require('scripts.brutal_morrowind.settings')
local storage = require('openmw.storage')
assert(type(world.applyObjectImpulse) == 'function', 'Brutal Morrowind requires native dynamic props')
return {
    engineHandlers = {onPlayerAdded = function(player)
        player:sendEvent('BrutalPhysicsAvailable', {available = true,
            superKick = type(world.applyNpcSuperKick) == 'function',
            objectGrab = type(world.holdObject) == 'function'})
    end},
    eventHandlers = {BrutalSettingsChanged = function(data)
        if data.player ~= world.players[1] or type(data.values) ~= 'table' then return end
        storage.globalSection(S.SYNC):set('values', data.values)
    end, BrutalPropImpulse = function(data)
        if not S.get('dynamicProps') then return end
        local item, attacker = data.item, data.attacker
        if not item or not item:isValid() or not item.enabled or not item.cell
            or not attacker or not attacker:isValid() or types.Actor.isDead(attacker)
            or item.cell ~= attacker.cell or not types.Item.objectIsInstance(item)
            or types.Light.objectIsInstance(item) or (item.position-attacker.position):length() > C.reach+20 then return end
        local direction = util.vector3(data.direction.x, data.direction.y, 0)
        if direction:length() < 0.1 then return end
        -- Morrowind weight is converted to a tuned gameplay mass, not claimed SI data.
        local record = item.type.record(item)
        local mass = math.max(0.25, math.min(50, (record.weight or 1) * 0.5))
        local velocity = direction:normalize() * math.min(700, 350/mass)
            + util.vector3(0, 0, math.min(180, 90/mass))
        world.applyObjectImpulse(item, velocity * mass, mass)
    end, BrutalGrabRequest = function(data)
        if not S.get('enabled') or not S.get('objectGrabEnabled') then return end
        local item, player = data.object, data.player
        if not item or not item:isValid() or not player or not player:isValid()
            or item.cell ~= player.cell or types.Actor.isDead(player) then return end
        world.holdObject(item, player, data.fragment)
    end, BrutalGrabRelease = function(data)
        if type(world.releaseHeldObject) ~= 'function' then return end
        local player = data.player
        if not player or not player:isValid() or not data.eyePos or not data.throwDir then return end
        world.releaseHeldObject(player, data.eyePos, data.throwDir, data.throwSpeed or 0, data.adrenaline == true)
    end, BrutalPlayObjectSound3d = function(data)
        if not S.get('combatSounds') or type(data) ~= 'table' or type(data.file) ~= 'string' then return end
        if data.file:sub(1, 15) ~= 'sound/brutal_dm/' or not vfs.fileExists(data.file) then return end
        local target = data.object
        if not target or not target:isValid() or not target.enabled or not target.cell then return end
        core.sound.playSoundFile3d(data.file, target, {
            volume = math.max(0.05, math.min(1, tonumber(data.volume) or 0.5)),
            pitch = math.max(0.5, math.min(2, tonumber(data.pitch) or 1)),
        })
    end},
}
