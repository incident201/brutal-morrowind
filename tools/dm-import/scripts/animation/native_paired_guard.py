"""Adapt native paired guard elbows while preserving both hand/item grips.

The owned game's paired grip/fingers remain untouched. Only the two upper/fore
joint frames are adapted to the native palm landmarks and a signed elbow plane.
A small common rigid correction may carry both hands and the item together.
Bounds are animation engineering limits, not clinical measurements.
"""
import numpy as np
from retarget_kf import norm, cross


def unit(v):
    return np.asarray(v, float) / norm(v)


def rotation(axis, radians):
    x, y, z = unit(axis)
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(radians)*k + (1-np.cos(radians))*(k@k)


def align(a, b):
    a, b = unit(a), unit(b)
    axis = cross(a, b)
    if norm(axis) < 1e-8:
        if a@b > 0: return np.eye(3)
        axis = cross(a, np.eye(3)[np.argmin(np.abs(a))])
    return rotation(axis, np.arctan2(norm(cross(a, b)), a@b))


def adapt_paired_elbows(nodes, parents, local, correction=None):
    def world_from(values):
        result = {}
        def get(name):
            if name not in result:
                result[name] = (get(parents[name]) if name in parents else np.eye(4)) @ values[name]
            return result[name]
        for name in nodes: get(name)
        return result
    world = world_from(local)
    translations = {name: value[:3,3].copy() for name,value in local.items()}
    correction=np.eye(4) if correction is None else correction
    before_hands = {s: correction@world[f'Bip01 {s} Hand'] for s in ('R', 'L')}
    before_item=correction@world['Weapon Bone']
    capsules = [(world['Bip01 Spine1'][:3,3], world['Bip01 Spine2'][:3,3], 9.),
                (world['Bip01 Pelvis'][:3,3]+[0,0,-4], world['Bip01 Pelvis'][:3,3]+[0,0,8], 10.),
                (world['Bip01 Head'][:3,3], world['Bip01 Head'][:3,3]+[0,0,7], 7.5)]
    records = {}
    for side in ('R', 'L'):
        names = [f'Bip01 {side} '+n for n in ('UpperArm','Forearm','Hand')]
        upper, fore, original_hand = [world[n] for n in names]
        hand=before_hands[side]
        shoulder, original_elbow, wrist = upper[:3,3],fore[:3,3],hand[:3,3]
        ua, fa = original_elbow-shoulder, original_hand[:3,3]-original_elbow
        ul, fl = norm(ua), norm(fa); ua, fa = unit(ua), unit(fa)
        source_hinge = unit(cross(ua,fa)); source_flex = np.arccos(np.clip(ua@fa,-1,1))
        axis=unit(wrist-shoulder);distance=norm(wrist-shoulder)
        if distance>=ul+fl-.01 or distance<=abs(ul-fl)+.01:
            raise ValueError('Unreachable paired hand '+side)
        along=(ul*ul-fl*fl+distance*distance)/(2*distance)
        center=shoulder+axis*along;radius=np.sqrt(max(0.,ul*ul-along*along))
        pole=original_elbow-center;pole=unit(pole-axis*(pole@axis))
        mcp=np.array([nodes[f'Bip01 {side} Finger{i}'].translation for i in (1,2)])
        palm=unit(mcp.mean(0));lateral=mcp[1]-mcp[0];lateral=unit(lateral-palm*(lateral@palm))
        pf=hand[:3,:3]@palm;pl=hand[:3,:3]@lateral;pn=cross(pf,pl)
        outward=shoulder-world['Bip01 Spine2'][:3,3];outward[2]=0;outward=unit(outward)
        def clearance(p,padding):
            return min(norm(p-a-(b-a)*np.clip((p-a)@(b-a)/max((b-a)@(b-a),1e-8),0,1))-r-padding
                       for a,b,r in capsules)
        best=None
        for degrees in range(-180,180,5):
            elbow=center+rotation(axis,np.radians(degrees))@pole*radius
            a,b=unit(elbow-shoulder),unit(wrist-elbow);hinge=unit(cross(a,b))
            swing=align(ua,a);moved=swing@source_hinge
            roll=np.arctan2(a@cross(moved,hinge),moved@hinge)
            flex=np.arccos(np.clip(a@b,-1,1))
            upper_r=rotation(a,roll)@swing@upper[:3,:3]
            fore_r=rotation(hinge,flex-source_flex)@rotation(a,roll)@swing@fore[:3,:3]
            palm_flex=np.degrees(np.arctan2(b@pn,b@pf));palm_side=np.degrees(np.arctan2(b@pl,b@pf))
            body=min(clearance(elbow,1.5),clearance(wrist,2.))
            inward=max(-(elbow-shoulder)@outward-ul*.25,0.)
            if abs(palm_flex)>55 or abs(palm_side)>30 or abs(np.degrees(roll))>65 or body<0 or inward>0:
                continue
            for pronation in (0,):
                fr=rotation(b,np.radians(pronation))@fore_r
                # Prefer the original sleeve orientation after the hinge/roll,
                # without moving the hand or any finger/weapon attachment.
                cost=(norm(elbow-original_elbow)**2*.015 + palm_flex*palm_flex*.002
                      + palm_side*palm_side*.004 + np.degrees(roll)**2*.001 + pronation*pronation*.002)
                if best is None or cost<best[0]:
                    best=(cost,elbow.copy(),upper_r.copy(),fr.copy(),dict(
                        elbow_swivel=degrees,upper_roll=float(np.degrees(roll)),fore_pronation=pronation,
                        elbow_flex=float(np.degrees(flex)),palm_flex=float(palm_flex),palm_side=float(palm_side),
                        body_clearance=float(body)))
        if best is None:
            raise ValueError('No natural native paired guard elbow for '+side)
        _, elbow, ur, fr, metrics = best
        u=np.eye(4);u[:3,:3]=ur;u[:3,3]=shoulder
        f=np.eye(4);f[:3,:3]=fr;f[:3,3]=elbow
        local[names[0]]=np.linalg.inv(world[parents[names[0]]])@u
        local[names[1]]=np.linalg.inv(u)@f
        local[names[2]]=np.linalg.inv(f)@hand
        # Bone translations are structural, never a reach/stretch parameter.
        for name in names:
            np.testing.assert_allclose(local[name][:3,3], translations[name], atol=2e-4)
        records[side]=metrics
        world=world_from(local)
    for side,before in before_hands.items():
        np.testing.assert_allclose(world[f'Bip01 {side} Hand'],before,atol=2e-4)
    np.testing.assert_allclose(world['Weapon Bone'],before_item,atol=2e-4)
    return local, records


def naturalize_paired_guard(nodes, parents, local):
    world={}
    def get(name):
        if name not in world:
            world[name]=(get(parents[name]) if name in parents else np.eye(4))@local[name]
        return world[name]
    for name in nodes:get(name)
    pivot=(world['Bip01 R Hand'][:3,3]+world['Bip01 L Hand'][:3,3])*.5
    blade=unit(world['Weapon Bone'][:3,1])
    angles=[(roll,yaw,pitch,advance) for roll in (0,-15,15,-30,30,-45,45)
            for yaw in (0,-15,15) for pitch in (0,-15,15) for advance in (0,6)]
    angles.sort(key=lambda a:abs(a[0])+abs(a[1])+abs(a[2])+a[3]*2)
    for roll,yaw,pitch,advance in angles:
        transform=np.eye(4);transform[:3,:3]=(rotation([0,0,1],np.radians(yaw))
            @rotation([1,0,0],np.radians(pitch))@rotation(blade,np.radians(roll)))
        transform[:3,3]=pivot+np.array([0,advance,0])-transform[:3,:3]@pivot
        try:
            adapted,metrics=adapt_paired_elbows(nodes,parents,{k:v.copy() for k,v in local.items()},transform)
        except ValueError:
            continue
        metrics['common_item_correction']={'roll':roll,'yaw':yaw,'pitch':pitch,'advance':advance}
        return adapted,metrics
    raise ValueError('No natural paired guard within modest common item adaptation')
