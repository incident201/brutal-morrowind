local settings=require('scripts.brutal_morrowind.settings')
local ambient = require('openmw.ambient')
local core = require('openmw.core')
local vfs = require('openmw.vfs')
local self = require('openmw.self')
local types = require('openmw.types')
local I = require('openmw.interfaces')
local loaded, assets = pcall(require, 'scripts.brutal_assets.effects')
local played={}
local impactSoundCooldowns={}
local function pickFile(category)
    local entry = loaded and assets.sounds[category]
    if not entry or not entry.files or #entry.files == 0 then return nil end
    local file = entry.files[math.random(#entry.files)]
    if vfs.fileExists(file) then return file end
    return nil
end
-- The player character's own voice: grunts on adrenaline strikes, throws and jumps.
-- Dark Messiah's hero is male; a female character uses the imported female grunts.
local HERO_VOICE={critical_voice=true,object_throw_voice=true}
local function voiceCategory(category)
    if not HERO_VOICE[category] or types.NPC.record(self).isMale then return category end
    return pickFile(category..'_female') and category..'_female' or category
end
local function cue(category, volume, pitch)
    if not settings.get('combatSounds') then return end
    if HERO_VOICE[category] and not settings.get('heroVoice') then return end
    local source = voiceCategory(category)
    local file = pickFile(source)
    if file then
        ambient.playSoundFile(file, {volume = volume, pitch=pitch or 1, scale = false})
        played[category]=(played[category] or 0)+1
        if source~=category then played[source]=(played[source] or 0)+1 end
    end
end
local function cue3d(category, target, volume, pitch)
    if not settings.get('combatSounds') then return end
    local file = pickFile(category)
    if not file then return end
    if target and target:isValid() and target ~= self then
        core.sendGlobalEvent('BrutalPlayObjectSound3d', {
            object = target, file = file, volume = volume or 0.5, pitch = pitch or 1,
        })
        played[category]=(played[category] or 0)+1
    else
        cue(category, volume or 0.5, pitch)
    end
end
local function classifyObjectMaterial(item)
    if not item or not item:isValid() then return 'object_hit_wood' end
    if types.Book.objectIsInstance(item) then return 'object_hit_book' end
    if types.Potion.objectIsInstance(item) then return 'object_hit_glass' end
    local ok, record = pcall(function() return item.type.record(item) end)
    local text = ((ok and record and ((record.name or '') .. ' ' .. (record.model or '')) or '') .. ' ' .. (item.recordId or '')):lower()
    if text:find('glass') or text:find('bottle') or text:find('flask') or text:find('vial') or text:find('beaker')
        or text:find('retort') or text:find('alembic') or text:find('calcinator') or text:find('lantern')
        or text:find('lamp') or text:find('crystal') or text:find('mirror') then
        return 'object_hit_glass'
    end
    if text:find('urn') or text:find('pottery') or text:find('ceramic') or text:find('clay') or text:find('vase')
        or text:find('amphora') or text:find('redware') or text:find('blueware') or text:find('plate')
        or text:find('platter') or text:find('bowl') or text:find('jug') or text:find('pitcher')
        or text:find('mug') or text:find('tankard') then
        return 'object_hit_pottery'
    end
    if text:find('skull') or text:find('bone') or text:find('stone') or text:find('rock') or text:find('ore')
        or text:find('brick') or text:find('mortar') or text:find('pestle') or text:find('gem')
        or text:find('ruby') or text:find('emerald') or text:find('diamond') or text:find('pearl') then
        return 'object_hit_stone'
    end
    if types.Weapon.objectIsInstance(item) or types.Armor.objectIsInstance(item)
        or types.Lockpick.objectIsInstance(item) or types.Probe.objectIsInstance(item)
        or types.Repair.objectIsInstance(item)
        or text:find('iron') or text:find('steel') or text:find('silver') or text:find('gold')
        or text:find('dwemer') or text:find('ebony') or text:find('daedric') or text:find('metal')
        or text:find('goblet') or text:find('chalice') or text:find('cup') or text:find('kettle')
        or text:find('cauldron') or text:find('pan') or text:find('pot') or text:find('candlestick')
        or text:find('censer') or text:find('fork') or text:find('spoon') or text:find('knife') then
        return 'object_hit_metal'
    end
    return 'object_hit_wood'
end
return {interfaceName='BrutalPlayerFeedback',interface={count=function(category)return played[category] or 0 end},eventHandlers = {
    -- Keep the boot impact audible at its original duration in bullet
    -- time, just as the kick animation itself runs at real-time speed.
    BrutalRagdollKickLanded = function() cue('kick_hit', 0.4) end,
    BrutalAdrenalineReady = function() cue('adrenaline_ready', 0.6) end,
    BrutalAdrenalineSpent = function() cue('critical_voice', 0.5,math.random(90,105)/100) end,
    BrutalAdrenalineSlowMotionStarted = function()
        if not I.UI.getMode() and not types.Actor.isDead(self)
            and types.Player.getControlSwitch(self,types.Player.CONTROL_SWITCH.Fighting) then
            cue('slow_motion_in',0.4)
        end
    end,
    BrutalAdrenalineSlowMotionEnded = function(e)
        if e.reason=='expired' then cue('slow_motion_out',0.2) end
    end,
    BrutalObjectGrabSound = function(e)
        if type(e)~='table' then return end
        if e.kind=='pickup' then cue('object_pickup', 0.55, math.random(95,105)/100)
        elseif e.kind=='drop' then cue('object_drop', 0.50, math.random(95,105)/100)
        elseif e.kind=='throw' then
            cue('object_throw_voice', e.adrenaline and 0.70 or 0.55, math.random(94,106)/100)
            cue('object_throw_whoosh', e.adrenaline and 0.60 or 0.45, math.random(95,105)/100)
        end
    end,
    BrutalThrownObjectHit = function(e)
        if type(e)~='table' then return end
        local pitch = math.random(92,108)/100
        cue3d('object_hit_flesh', e.target or e.item, e.adrenaline and 0.80 or 0.65, pitch)
        if not e.item or not types.Actor.objectIsInstance(e.item) then
            cue3d(classifyObjectMaterial(e.item), e.target or e.item, 0.45, pitch)
        end
    end,
    BrutalDynamicObjectImpact = function(e)
        if type(e)~='table' or not e.item or not e.item:isValid() then return end
        local t = core.getRealTime()
        local id = e.item.id
        if impactSoundCooldowns[id] and t < impactSoundCooldowns[id] then return end
        impactSoundCooldowns[id] = t + 0.09
        local vol = math.max(0.20, math.min(0.68, ((e.speed or 45) - 45) / 420 + 0.20))
        cue3d(classifyObjectMaterial(e.item), e.item, vol, math.random(90,110)/100)
    end,
}}
