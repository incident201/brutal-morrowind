local settings=require('scripts.brutal_morrowind.settings')
-- Dark Messiah gait selection for the player. Alt and overencumbrance select
-- walking. The engine owns the speeds and applies sprint fatigue and recovery.
local self = require('openmw.self')
local types = require('openmw.types')
local input = require('openmw.input')
require('scripts.brutal_morrowind.input_bindings')
local I = require('openmw.interfaces')
local C = require('scripts.brutal_morrowind.config')

local setGait = types.Actor.setKineticGait
local WALK, RUN, SPRINT = 1, 2, 3
local gait, sprinting, shiftWasDown = nil, false, false
local scripted -- when set, {shift=, alt=} replaces the keyboard state

local function fatigueFraction()
    local fatigue = types.Actor.stats.dynamic.fatigue(self)
    return fatigue.base > 0 and fatigue.current / fatigue.base or 0
end

local function update()
    if not setGait then return end
    if not settings.get('movementEnabled') then
        if gait~=nil then setGait(self,0) end
        gait,sprinting,shiftWasDown=nil,false,false
        return
    end
    local shift, alt = false, false
    if scripted then
        shift, alt = scripted.shift == true, scripted.alt == true
    elseif not I.UI.getMode() then
        shift, alt = input.getBooleanActionValue('BrutalSprint'), input.isAltPressed()
    end
    if I.BrutalClinch and I.BrutalClinch.status().active then shift,alt=false,false end
    local fraction = fatigueFraction()
    local staminaEnabled = settings.get('staminaEnabled')
    local overloaded = types.Actor.getEncumbrance(self) > types.Actor.getCapacity(self)
    local needed = C.sprintStaminaNeeded / C.staminaMax
    -- The reserve is needed only to start. An active sprint can use all remaining
    -- stamina; once exhausted, the sprint button must be released to start again.
    if not shift or overloaded or (staminaEnabled and fraction <= 0) or types.Actor.isSwimming(self) then
        sprinting = false
    elseif not shiftWasDown and (not staminaEnabled or fraction >= needed) then
        sprinting = true
    end
    shiftWasDown = shift
    local wanted = sprinting and SPRINT or ((alt or overloaded) and WALK or RUN)
    if wanted ~= gait then
        gait = wanted
        setGait(self, gait)
    end
end

return {
    interfaceName = 'BrutalStamina',
    interface = {
        version = 1,
        status = function()
            return {gait = gait, sprinting = sprinting, fraction = fatigueFraction()}
        end,
        setScriptedInput = function(keys) scripted = keys end,
    },
    engineHandlers = {
        onUpdate = update,
        -- The engine does not save the gait; choose it again on the next update.
        onLoad = function() gait, sprinting, shiftWasDown = nil, false, false end,
    },
}
