local postprocessing=require('openmw.postprocessing')
local core=require('openmw.core')
local ui=require('openmw.ui')
local shader,checked=nil,false
local lossAt,previousId=nil,nil
local size,weight=nil,0
local warmFrames=0
local active=false
local function setActive(value)
    if shader and active~=value then shader:setActive(value) end
    active=value
end
local function draw(state,visible)
    -- Menus suspend presentation without restarting the same loss on resume.
    if not visible then
        weight=0
        if shader then shader:setFloat('brutalClinchHistory',0) end
        setActive(false)
        warmFrames=0
        return
    end
    if not checked then
        checked=true;shader=postprocessing.load('brutal_clinch_blur')
    end
    if not shader then return end
    local newlyEnabled=not shader:isEnabled()
    if newlyEnabled then
        shader:setFloat('brutalClinchHistory',0);shader:enable();warmFrames=0
    end
    local screen=ui.screenSize()
    if size and (screen.x~=size.x or screen.y~=size.y) then warmFrames=0 end
    size=screen
    if state.phase=='lose' and state.id~=previousId then
        lossAt,previousId=core.getSimulationTime(),state.id
        warmFrames=0
    end
    local remaining=lossAt and 1.4-(core.getSimulationTime()-lossAt) or 0
    -- Warm once after enabling, then keep the allocated chain suspended until
    -- a loss. Zero-weight frames overwrite stale history before accumulation.
    local needed=remaining>.6 or newlyEnabled or (active and warmFrames<4)
    if needed and not active then warmFrames=0 end
    setActive(needed)
    if not needed then weight=0;return end
    warmFrames=warmFrames+1
    weight=warmFrames>=4 and math.min(192,math.max(0,(remaining-.6)*1280))/255 or 0
    shader:setFloat('brutalClinchHistory',weight)
end
local function destroy()
    lossAt,previousId=nil,nil
    weight,warmFrames=0,0
    setActive(false)
    if shader then
        shader:setFloat('brutalClinchHistory',0)
        if shader:isEnabled() then shader:disable() end
    end
end
return {draw=draw,destroy=destroy,status=function()
    return {weight=weight,loaded=shader~=nil,active=active}
end}
