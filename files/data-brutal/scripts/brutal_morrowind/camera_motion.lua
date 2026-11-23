-- Dark Messiah's first-person camera motion (player_head_mvt, mm_headbobbing).
-- Movement: the walk/run clip of the 8-way movement direction, the sprint clip and the
-- landing tilt, blended like DM's single head entity (optional, on by default).
-- DM adds these clips to an eye that already rides its body's head. Here the engine
-- puts the eye on the visible torso, so its steps, crouch and sprint bob come from
-- that body; the movement clips add only their head turns (strafe roll, backward
-- pitch, landing). Their translation (up to 17 units sideways while sprinting, on a
-- clock of its own) would swing the view across a torso that does not sway.
-- Actions: the head clip DM plays with each arm action (kicks, parries, charged swings,
-- rushes, draws...), found between the text keys of the playing arm group, so it
-- follows re-timed groups. The clinch keeps its own curve in clinch.lua.
local self=require('openmw.self')
local core=require('openmw.core')
local types=require('openmw.types')
local camera=require('openmw.camera')
local animation=require('openmw.animation')
local util=require('openmw.util')
local I=require('openmw.interfaces')
local settings=require('scripts.brutal_morrowind.settings')
local loaded,data=pcall(require,'scripts.brutal_assets.camera_head')
if not loaded or type(data)~='table' or not data.clips then data=nil end

local FADE=.2 -- Source sequence transition
local LAND_AIR=.3 -- Airborne at least this long before a landing plays its clip
local GROUND_FINISHER='brutal_ground_finisher'
local BACKSTAB_OUT='brutal_backstab_out'
local DIRECTIONS={'N','NE','E','SE','S','SW','W','NW'}
local ZERO={0,0,0,0,0,0}

-- The spear uses the same action mixer as the imported head layers. Each
-- curve segment runs between two arm text keys, so weapon speed, an
-- interrupted combo and a held charge all use the actual game clock.
-- Amplitudes follow DM's own staff thrust (bigslash_staff_B_out: -14 pitch,
-- -20 roll into the drive, +15 pitch rebound) and shield bash (-19/-17 in
-- .13 s), with the pitch sign turned so that the drive dips the eye into
-- the line instead of lifting it off the target (DM's negative pitch looks
-- up): the chamber cocks the head back, the drive dives into the line,
-- contact adds a short kick, then the head rebounds and settles.
-- Values: pitch, yaw, roll in degrees; x right, y forward, z up eye offset.
-- The eye offset stays zero: the arm KF already carries the eye and the arms
-- together on the lunge, and moving the eye alone toward the arms pushes the
-- front hand below the bottom edge of the view. The pitch never looks up for
-- the same reason. Pitch and yaw stay small (2-4 and 1-2 degrees): the arms do
-- not follow these camera extras, so a large pitch or yaw lifts the front hand
-- and swings the point off the aim on screen. Roll turns about the line of
-- sight, keeps the point on the aim and carries most of the impact.
local function addSpearHeadMotion(set, clips)
    local function add(group, from, to, a, b, shape)
        local name='spear_'..group..'_'..from
        local frames={}
        for i=0,12 do
            local u=i/12
            local w
            if shape=='drive' then w=u*u*(2.6-1.6*u)          -- fastest at contact
            elseif shape=='snap' then w=1-(1-u)^3             -- immediate kick
            else w=u*u*(3-2*u) end
            local frame={0,0,0,0,0,0}
            for k=1,3 do frame[k]=math.rad(a[k]+(b[k]-a[k])*w) end
            for k=4,6 do frame[k]=a[k]+(b[k]-a[k])*w end
            frames[#frames+1]=frame
        end
        clips[name]={fps=12,loop=false,frames=frames}
        set[#set+1]={start=group..' '..from,stop=group..' '..to,clip=name,duration=1}
    end
    -- Keep the point readable through the fast drive. The eye dips briefly as
    -- the forward foot takes the weight, then settles while the hands return.
    local quick={
        fasta={cock={0.2,-0.15,0.55,0,0,0},drive={1.6,0.3,-1.8,0,0,0},kick={3.1,0.45,-3.1,0,0,0},rebound={0.6,0,0.30,0,0,0}},
        fastb={cock={0.2,-0.10,0.45,0,0,0},drive={1.4,0.2,-1.5,0,0,0},kick={2.8,0.35,-2.7,0,0,0},rebound={0.5,0,0.20,0,0,0}},
        fastc={cock={0.25,-0.20,0.65,0,0,0},drive={1.7,0.4,-2.0,0,0,0},kick={3.3,0.55,-3.4,0,0,0},rebound={0.6,0,0.35,0,0,0}},
        fastd={cock={0.30,-0.25,0.80,0,0,0},drive={1.9,0.5,-2.4,0,0,0},kick={3.6,0.65,-4.0,0,0,0},rebound={0.7,0,0.45,0,0,0}},
    }
    quick.fastloopa=quick.fasta
    for _,group in ipairs({'fasta','fastloopa','fastb','fastc','fastd'}) do
        local q=quick[group]
        add(group,'start','plant',ZERO,q.cock)
        add(group,'plant','thrust hit',q.cock,q.drive,'drive')
        add(group,'thrust hit','weight forward',q.drive,q.kick,'snap')
        add(group,'weight forward','settle end',q.kick,q.rebound)
        add(group,'settle end','stop',q.rebound,ZERO)
    end
    -- Charged: a long cock-back held while charging, then DM's staff lunge.
    local charged={0.5,-1.5,3.85,0,0,0}
    local inhale={0.5,-1.7,4.29,0,0,0}
    local drive={2.4,1.88,-7.7,0,0,0}
    local kick={3,2.12,-9.35,0,0,0}
    local rebound={0.5,-1.25,-2.75,0,0,0}
    add('thrust','start','max attack',ZERO,charged)
    add('thrust hold','start','breath',charged,inhale)
    add('thrust hold','breath','stop',inhale,charged)
    add('thrust','plant','hit',charged,drive,'drive')
    add('thrust','hit','weight forward',drive,kick,'snap')
    add('thrust','weight forward','settle end',kick,rebound)
    add('thrust','settle end','large follow stop',rebound,ZERO)
end
if data and data.sets and data.sets.brutal_fp_spear then
    addSpearHeadMotion(data.sets.brutal_fp_spear.weapononehand, data.clips)
end

local function sample(clip,t)
    local frames=clip.frames
    local n=#frames
    if n==1 then return frames[1] end
    local position=t*clip.fps
    if clip.loop then position=position%(n-1) else position=math.max(0,math.min(n-1,position)) end
    local i=math.floor(position)
    local a,b=frames[i+1],frames[math.min(i+2,n)]
    local f=position-i
    local out={}
    for k=1,6 do out[k]=a[k]+(b[k]-a[k])*f end
    return out
end
local function length(clip) return (#clip.frames-1)/clip.fps end
local function mix(a,b,w)
    local out={}
    for k=1,6 do out[k]=a[k]+(b[k]-a[k])*w end
    return out
end

-- Movement layer: one weight per clip, faded towards the current target.
local weights,phases={},{}
local landing,landingTime,airborne=nil,0,0
local function movementTarget()
    if not types.Actor.isOnGround(self) or types.Actor.isSwimming(self) then return nil end
    local forward,side=self.controls.movement,self.controls.sideMovement
    if math.abs(forward)<.1 and math.abs(side)<.1 then return nil end
    if types.Actor.getCurrentSpeed(self)<1 then return nil end
    local gait=I.BrutalStamina and I.BrutalStamina.status().gait
    if gait==3 and forward>.3 then
        local weapon=types.Actor.getEquipment(self)[types.Actor.EQUIPMENT_SLOT.CarriedRight]
        local kind=weapon and types.Weapon.objectIsInstance(weapon) and types.Weapon.record(weapon).type
        if kind==types.Weapon.TYPE.SpearTwoWide or kind==types.Weapon.TYPE.BluntTwoWide then return '@megasprint_staff' end
        if kind==types.Weapon.TYPE.ShortBladeOneHand then return '@megasprint_daggers' end
        return '@megasprint_sword'
    end
    local running
    if gait then running=gait>=2 else running=self.controls.run end
    if self.controls.sneak then running=false end
    -- DM's 0..315 degree sectors: N forward, E right, S back, W left.
    local angle=math.atan2(side,forward)
    local sector=math.floor((angle/(math.pi/4))+.5)%8
    return (running and '@run_' or '@walk_')..DIRECTIONS[sector+1]
end
local function movementSample(dt)
    local target=movementTarget()
    local total,out=0,{0,0,0,0,0,0}
    for name,clip in pairs(data.clips) do
        if name:find('^@walk_') or name:find('^@run_') or name:find('^@megasprint_') then
            local w=weights[name] or 0
            w=name==target and math.min(1,w+dt/FADE) or math.max(0,w-dt/FADE)
            weights[name]=w
            if w>0 then
                phases[name]=(phases[name] or 0)+dt
                local s=sample(clip,phases[name])
                for k=1,3 do out[k]=out[k]+s[k]*w end
                total=total+w
            else phases[name]=0 end
        end
    end
    if total>1 then for k=1,6 do out[k]=out[k]/total end end
    -- Landing after a real fall: jump_out_* tilts the view on its own clock.
    local ground=types.Actor.isOnGround(self) or types.Actor.isSwimming(self)
    if not ground then airborne=airborne+dt
    else
        if airborne>=LAND_AIR then
            local gait=I.BrutalStamina and I.BrutalStamina.status().gait
            local moving=target~=nil
            landing=(gait==3 and moving) and '@jump_out_sprint' or (moving and (target:find('^@run_') and '@jump_out_run' or '@jump_out_walk')) or '@jump_out_idle'
            landingTime=0
        end
        airborne=0
    end
    if landing and data.clips[landing] then
        local clip=data.clips[landing]
        landingTime=landingTime+dt
        if landingTime>=length(clip) then landing=nil
        else
            local s=sample(clip,landingTime)
            for k=1,6 do out[k]=out[k]+s[k] end
        end
    end
    return out
end

-- Action layer: the arm group playing now and its part between two text keys.
local function weaponKind(weapon)
    return weapon and types.Weapon.objectIsInstance(weapon) and types.Weapon.record(weapon).type
end
local function folder(ground)
    local equipment=types.Actor.getEquipment(self)
    local left=equipment[types.Actor.EQUIPMENT_SLOT.CarriedLeft]
    local kind=weaponKind(equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight])
    local W=types.Weapon.TYPE
    local staff=kind==W.SpearTwoWide or kind==W.BluntTwoWide
    if kind==W.SpearTwoWide then return 'brutal_fp_spear' end
    if kind==W.LongBladeTwoHand then return 'brutal_fp_paired' end
    if kind==W.AxeTwoHand or kind==W.BluntTwoClose then return 'brutal_fp_paired_blunt' end
    if ground then
        -- A hidden, equipped shield does not select the sword-and-shield rig.
        if staff then return 'brutal_fp_staff' end
    end
    if left and types.Armor.objectIsInstance(left) and types.Armor.record(left).type==types.Armor.TYPE.Shield then
        return 'brutal_fp_shield'
    end
    if staff then return 'brutal_fp_staff' end
    if kind==W.ShortBladeOneHand and (not ground or
        (settings.get('dualWieldEnabled') and weaponKind(left)==W.ShortBladeOneHand)) then
        return 'brutal_fp_dagger'
    end
    return 'brutal_fp'
end
local function groupSample(group,parts,t)
    for _,part in ipairs(parts) do
        local s=animation.getTextKeyTime(self,group..': '..part.start)
        local e=animation.getTextKeyTime(self,group..': '..part.stop)
        if s and e and e>s and t>=s and t<=e then
            local clip=data.clips[part.clip]
            if clip then
                return sample(clip,math.min((t-s)/(e-s)*part.duration,length(clip)))
            end
        end
    end
    return nil
end
local function actionSample()
    local sets=data.sets or {}
    local backstabTime=animation.getCurrentTime(self,BACKSTAB_OUT)
    if backstabTime then
        local daggerSet=sets.brutal_fp_dagger or {}
        local parts=daggerSet[BACKSTAB_OUT]
        if parts then return groupSample(BACKSTAB_OUT,parts,backstabTime),true end
    end
    local groundTime=animation.getCurrentTime(self,GROUND_FINISHER)
    local set,base=sets[folder(groundTime~=nil)] or {},sets.brutal_fp or {}
    if groundTime then
        -- The finisher starts from an armed hold. Sample its own active phase
        -- even while a charged attack is blending out underneath it.
        local parts=set[GROUND_FINISHER] or base[GROUND_FINISHER]
        if parts then return groupSample(GROUND_FINISHER,parts,groundTime),true end
    end
    for _,groups in ipairs({set,base}) do
        for group,parts in pairs(groups) do
            local t=animation.getCurrentTime(self,group)
            if t and (groups==set or not set[group]) then
                local motion=groupSample(group,parts,t)
                if motion then return motion,false end
            end
        end
    end
    return nil
end

local current={0,0,0,0,0,0}
local actionWeight,lastAction,groundAction=0,ZERO,false
local pulseState=nil
return {
    interfaceName='BrutalCameraMotion',
    interface={version=2,pulse=function(pitch,roll,z,duration)
        if type(duration)~='number' or duration<=0 then return end
        pulseState={pitch=tonumber(pitch) or 0,roll=tonumber(roll) or 0,z=tonumber(z) or 0,duration=duration,elapsed=0}
    end,status=function()
        local moving={}
        for name,w in pairs(weights) do if w>0 then moving[name]=w end end
        return {available=data~=nil,movement=moving,landing=landing,actionWeight=actionWeight,
            pitch=current[1],yaw=current[2],roll=current[3],x=current[4],y=current[5],z=current[6]}
    end},
    engineHandlers={
        onFrame=function(dt)
            current={0,0,0,0,0,0}
            if not data or not settings.get('enabled') then return end
            if camera.getMode()~=camera.MODE.FirstPerson or I.UI.getMode() or types.Actor.isDead(self) then return end
            if settings.get('firstPersonAnimations')~='darkMessiah' then return end
            if I.BrutalClinch and I.BrutalClinch.status().active then return end
            -- Paused frames: the omw camera does not reset its extras, so adding again would accumulate.
            if dt<=0 then return end
            dt=math.min(dt,.1)
            local out=settings.get('cameraMotionMovement') and movementSample(dt) or {0,0,0,0,0,0}
            local action,ground=actionSample()
            -- Actions replace the movement clip, with DM's .2 s transitions either way.
            if action then lastAction=action;groundAction=ground;actionWeight=math.min(1,actionWeight+dt/FADE)
            else actionWeight=math.max(0,actionWeight-dt/FADE) end
            if actionWeight>0 then out=mix(out,lastAction,actionWeight) end
            if actionWeight>0 and groundAction then
                -- The authored layer adds a steep dip to an already lowered aim.
                -- Stop the combined view at the vertical instead of turning over.
                local pitch=camera.getPitch()+camera.getExtraPitch()
                local limit=math.pi/2-.001
                out[1]=math.max(math.min(out[1],math.max(0,limit-pitch)),math.min(0,-limit-pitch))
            end
            if pulseState then
                pulseState.elapsed=pulseState.elapsed+dt
                if pulseState.elapsed>=pulseState.duration then
                    pulseState=nil
                else
                    local w=math.sin(math.pi*pulseState.elapsed/pulseState.duration)
                    out[1]=out[1]+pulseState.pitch*w
                    out[3]=out[3]+pulseState.roll*w
                    out[6]=out[6]+pulseState.z*w
                end
            end
            current=out
            camera.setExtraPitch(camera.getExtraPitch()+out[1])
            camera.setExtraYaw(camera.getExtraYaw()+out[2])
            camera.setExtraRoll(camera.getExtraRoll()+out[3])
            camera.setFirstPersonOffset(camera.getFirstPersonOffset()+util.vector3(out[4],out[5],out[6]))
        end,
        onLoad=function() weights,phases,landing,airborne,actionWeight,groundAction,pulseState={},{},nil,0,0,false,nil end,
    },
}
