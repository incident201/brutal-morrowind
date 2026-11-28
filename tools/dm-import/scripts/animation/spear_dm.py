#!/usr/bin/env python3
"""Build first-person spear strokes from the second jab of DM's charged staff attack.

Forward jabs start in the same right-side guard and aim along several lines.
Both palms keep their orientation and finger curl. The shaft slides a short
distance through the lead palm only during a charged draw. Sidesteps use
the donor's lateral swings.
The running carry moves the closed grip with the player's stride.
The rear hand regrips on parry. Draw and sheathe join the guard on continuous paths.
"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

import numpy as np

from import_two_hand_viewmodel import (
    Grasp, ITEM, Pose, Rig, basis, inverse, keys, lerp, load, matrix, matrix_quat, native_starts, nif, quat_matrix,
    sample, slerp, tracks, turn_about, unit,
)
from arm_steering import LocalPose, blend, ease, steer_arm
from import_staff_native_grasp import swing as rotate_between

# The straight jab inside DM's charged staff attack, relative to its hit2 key.
READY_BEFORE_HIT = .032   # both palms on the shaft, elbows still bent
JAB_END = .06             # brief impact lock after charged hit before crisp recovery
QUICK_CHAMBER = 4. / 60.  # exact 60Hz frame: snappy pull-back & strike-line alignment
QUICK_DRIVE = 6. / 60.    # exact 60Hz frame: explosive forward thrust (hit at 0.167s)
QUICK_FOLLOW = .028       # short stiff follow-through braking after contact
QUICK_SETTLE = .040       # compact impact settle (no dead midair pause)
QUICK_RETURN = .235       # crisp elastic recovery back to guard
QUICK_CONTACT = .915      # contact while tip still carries high forward speed
CHARGED_DRIVE = .125      # violent accelerating lunge to contact (~260 u/s at hit)
CHARGED_FOLLOW = .038     # impact penetration/brake after hit
CHARGED_RETURN = .36      # strong controlled recovery (no dead tail)
CHARGE_DRAW_SLIDE = 6.5   # shaft slides back through braced lead palm while coiling
CHARGE_DRIVE_SLIDE = -6.8 # shaft shoots forward through lead palm on full lunge (keeps hands cleanly spaced)
QUICK_DRAW_SLIDE = 4.0    # shaft pulls back through lead palm during micro-chamber
QUICK_DRIVE_SLIDE = -5.4  # shaft slides forward through lead palm on quick thrust
RATE = 60.
# A charged release moves the wrist through its most tightly bent pose quickly.
# Extra samples keep the shaft and closed hand together between rotation keys.
CHARGED_SAMPLE_RATE = 3 * RATE
SWEEPS = {
    # Use the two passes across the front in the staff's charged slash. The
    # quick staff combo twirls the haft behind the player between attacks.
    'faststrafeleft': ('slashright', 'slash hit2', .136, .060),
    'faststraferight': ('slash', 'slash hit', .106, .070),
}
SWEEP_ENTRY, SWEEP_RETURN = .35, .46
SWEEP_LEAD, SWEEP_TAIL = .065, .112
SWEEP_DIP = (0., 12., -5.)            # clear the view without folding the rear arm against its shoulder
# The staff's own sections (side swing, draw, sheathe) start and end in the
# staff guard, haft across the view with the point up and left. Blending from
# the spear guard straight into it swings the haft through the camera in a
# few frames. DM's side swing passes through a point-forward pose twice: the
# early wind-up (SWING_JOIN) and the recovery after the second impact
# (SWING_LEAVE). The spear joins and leaves the swing there.
SWING_JOIN, SWING_LEAVE = .5, .24     # after 'slash start' / after 'slash hit2'
JOIN, LEAVE = .53, .54                # blend lengths into and out of the swing
DIP = (0., 6., -16.)                  # the hands' detour under the eye during those blends
DRAW_EARLY = .1                       # leave DM's draw this long before the attach key
# Tight, realistic human aim cluster around the crosshair at contact (rig units
# at the far aim plane, ~1.5-3.0 deg off-center instead of robotic 1-pixel or
# sloppy 10-deg sideways flop).
AIM = {'fasta': (1.8, 3.0), 'fastb': (-3.6, 2.0), 'fastc': (-1.4, -3.4),
       'fastd': (3.6, -1.6), 'fastloopa': (-2.2, 3.4),
       'thrust': (0., 0.), 'neutral': (0., -.1)}
# Distinct chamber offsets (anchor_x, anchor_draw, anchor_z, axial_roll_deg)
# so each quick thrust attacks from its own martial angle along a locked line.
CHAMBERS = {
    'fasta':     (1.2, 7.2, -0.4,  -4.),   # crisp centerline-right straight
    'fastb':     (2.6, 7.6,  2.3,   5.),   # high-right shoulder thrust downward-in
    'fastc':     (2.2, 7.6, -3.4,  -8.),   # low-right hip thrust upward-in
    'fastd':     (4.4, 7.8,  0.8,   6.),   # wide outside-right sharp inward thrust
    'fastloopa': (1.4, 6.8, -1.5,  -3.),   # tight low-inside rising thrust
    'thrust':    (2.2, 12.0, -2.4, -6.),   # deep coiled heavy lunge
    'neutral':   (2.2, 12.0, -2.4, -6.),
}


def recovery_clock(elapsed, duration):
    u = min(1., max(0., elapsed / duration))
    return SWEEP_TAIL * (1. - (1. - u)**(duration / SWEEP_TAIL))


def sweep_clock(elapsed, start, stop):
    """Donor time and weight, with a matching clock speed on both joins."""
    if elapsed < SWEEP_ENTRY:
        u = max(0., elapsed / SWEEP_ENTRY)
        return start - SWEEP_LEAD * (1. - u**(SWEEP_ENTRY / SWEEP_LEAD)), ease(u)
    elapsed -= SWEEP_ENTRY
    if elapsed <= stop - start:
        return start + elapsed, 1.
    recovery = elapsed - (stop - start)
    return stop + recovery_clock(recovery, SWEEP_RETURN), 1. - ease(recovery / SWEEP_RETURN)


def sampled_poses(segments):
    samples = {}
    for begin, stop, fn, rate in segments:
        for t in np.linspace(begin, stop, max(2, int(np.ceil((stop - begin) * rate)) + 1)):
            samples[float(t)] = fn(float(t - begin))
    return samples


def replace_frames(stream, samples, intervals, stop_time=None):
    """Merge generated frames into the donor's controllers."""
    times = np.array(sorted(samples))
    names = (e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData))
    for name, controller in zip(names, stream.root.controllers):
        if name not in samples[times[0]]:
            continue
        for index, field in enumerate((controller.data.rotations, controller.data.translations)):
            original = field.keys
            if original is None or not len(original):
                continue
            keep = np.ones(len(original), dtype=bool)
            for begin, stop in intervals:
                keep &= (original[:, 0] < begin - 1e-5) | (original[:, 0] > stop + 1e-5)
            values = [matrix_quat(samples[t][name][:3, :3]) if index == 0 else samples[t][name][:3, 3]
                      for t in times]
            merged = np.vstack((original[keep], np.column_stack((times, values)))).astype(np.float32)
            merged = merged[np.argsort(merged[:, 0], kind='stable')]
            if index == 0:
                for i in range(1, len(merged)):
                    if merged[i - 1, 1:] @ merged[i, 1:] < 0.:
                        merged[i, 1:] *= -1
            field.keys = merged
        if stop_time is not None:
            controller.stop_time = max(controller.stop_time, stop_time)


def retime_gait(stream, ratio):
    """Keep the first-person gait on the same loop clock as the visible body."""
    text = keys(stream)
    text.keys = np.array([(time * ratio, label) for time, label in text.keys], dtype=text.keys.dtype)
    for controller in stream.root.controllers:
        controller.start_time *= ratio
        controller.stop_time *= ratio
        for channel in (controller.data.rotations, controller.data.translations, controller.data.scales):
            if channel is not None and channel.keys is not None and len(channel.keys):
                scaled = channel.keys.copy()
                scaled[:, 0] *= ratio
                channel.keys = scaled


def body_footwork(body, weapon):
    """Fit the imported walking footfalls to the spear's actual contact keys."""
    folder = body / 'Animations/brutal_firstperson'
    idle = folder / 'idle1h.kf'
    walking = folder / 'walkforward1h.kf'
    stride = tracks(walking)
    period = native_starts(walking)['walkforward1h: loop stop']
    stance = tracks(idle)
    stream = load(idle)
    text = keys(stream)
    marks = {name.removeprefix('weapononehand: '): time for name, time in native_starts(weapon).items()}
    group = 'brutal_spear_step: '
    ranges = {}
    for clip in ('fasta', 'fastb', 'fastc', 'fastd', 'fastloopa'):
        start, plant, hit, weight, stop = [marks[clip+' '+key] for key in
                                          ('start','plant','thrust hit','weight forward','stop')]
        lead = .5 if clip in ('fastb', 'fastd') else 0.
        ranges[clip] = [(start, lead-.13, 0.), (plant, lead-.05, .36),
                        (hit, lead+.19, .72), (weight, lead+.25, .84),
                        (hit+QUICK_SETTLE, lead+.29, .82), (stop, lead+.48, 0.)]
    for letter in 'abcd':
        name = 'fastend'+letter
        ranges[name] = [(marks[name+' start'], 0., 0.), (marks[name+' stop'], 0., 0.)]
    draw, back, release, hit, end = [marks['thrust '+key] for key in
                                     ('start','max attack','min hit','hit','large follow stop')]
    ranges['thrust'] = [(draw, .65, 0.), (back, .78, .45), (release, .78, .45),
                        (hit, 1.22, .9), (hit+.13, 1.30, .86), (end, 1.53, 0.)]
    ranges['thrust hold'] = [(marks['thrust hold '+key], .78, .45) for key in ('start','stop')]
    labels = [(time, group+label) for label,time in marks.items()
              if label.startswith(('fast', 'thrust ')) and not label.startswith('faststrafe')]
    # Secondary animation.play() resolves the hold's generic loop range.
    for key in ('loop start','loop stop'):
        labels.append((marks['thrust hold '+key], group+key))
    final_time = max(t for t,_ in labels)
    labels.extend([(0., group+'start'), (final_time, group+'stop')])
    text.keys = np.array(sorted(labels), dtype=text.keys.dtype)
    frames = {}
    for curve in ranges.values():
        for (begin,p0,w0), (end,p1,w1) in zip(curve,curve[1:]):
            for t in np.linspace(begin,end,max(2,int(np.ceil((end-begin)*60))+1)):
                u = ease((t-begin)/(end-begin))
                frames[t] = (lerp(p0,p1,u)%1.*period, lerp(w0,w1,u))
    times = sorted(frames)
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    for name, ctl in zip(names, stream.root.controllers):
        for index, field in enumerate((ctl.data.rotations, ctl.data.translations)):
            n, interpolate = (4, slerp) if index == 0 else (3, lerp)
            rest = stance[name][index][0,1:]
            values = []
            for t in times:
                phase, weight = frames[t]
                donor = sample(stride[name][index], phase, n, interpolate)
                value = interpolate(rest, donor, weight)
                if index == 0 and values and value @ values[-1] < 0.:
                    value = -value
                values.append(value)
            field.keys = np.column_stack((times,values)).astype(np.float32)
        ctl.start_time, ctl.stop_time = 0., final_time
    stream.save(folder / 'brutal_spear_step.kf')


def build(arms, body=None):
    folder = arms / 'Animations/brutal_fp_staff'
    source = folder / 'weapononehand.kf'
    target = folder.with_name('brutal_fp_spear')
    target.mkdir(exist_ok=True)
    stream = load(source)
    text = keys(stream)
    marks = {line.removeprefix('weapononehand: '): float(t)
             for t, label in text.keys for line in label.splitlines()}
    channels = tracks(source)               # source data, sampled below
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    animated = list(channels)

    hit2 = marks['thrust hit2']
    ready_time = hit2 - READY_BEFORE_HIT

    def at(t, source_channels=channels):
        pose = Pose(rig, source_channels, t)
        return {n: pose.local(n) for n in animated}

    segments = []          # (start, stop, fn(elapsed) -> local transforms, sample rate)
    labels = []

    def section(start, length, fn, name, events, rate=RATE):
        segments.append((start, start + length, fn, rate))
        labels.extend((start + t, 'weapononehand: ' + name + ' ' + e) for t, e in events)

    contact_pose = Pose(rig, channels, hit2)
    grasps = {side: Grasp(contact_pose, side, False) for side in 'RL'}

    # The calibrated native grasp places the center of the shaft inside the
    # curl of the fingers. Finger joints on the right rig mirror their local Z.
    profile = json.loads((arms / 'staff-native-grasp-profile.json').read_text())
    contact_item = contact_pose.world(ITEM)
    mirror_local = np.diag((1., 1., -1.))
    mirror_world = np.diag((-1., 1., 1.))
    shaft_axis = np.array((0., 1., 0.))
    # Extra local flexion of base, middle and tip joints for a slim shaft.
    # Measured against the iron spear and steel halberd in the native hand.
    finger_curl = ((16., -15., -24.), (23., -5., 24.), (10., 24., 0.),
                   (-3., 32., -3.), (7., 22., -10.))
    # Oppose each thumb toward the index finger so the pad lies against
    # the shaft. Curling only about local Z leaves the tip jutting rearward.
    thumb_opposition = (-21., 3., 13.)
    palm_anchor = np.array(profile['shaft_anchor_native_hand']) + (.44, -.87, 0.)
    lead_station = -7.
    for side, station, roll in (('L', lead_station, 158.), ('R', -26., -40.)):
        grasp = grasps[side]
        reflection = mirror_local if side == 'R' else np.eye(3)
        rotation = turn_about(shaft_axis, roll) @ turn_about((1., 0., 0.), 180.) @ grasp.grip[:3, :3]
        axis = rotation @ reflection @ profile['shaft_axis_native_hand_reference']
        rotation = rotate_between(axis, shaft_axis * np.sign(axis[1])) @ rotation
        anchor = rotation @ reflection @ palm_anchor
        grasp.grip = matrix(rotation, station * shaft_axis - anchor)
        for digit, bends in enumerate(finger_curl):
            for joint, (suffix, degrees) in enumerate(zip(('', '1', '2'), bends)):
                name = f'Bip01 {side} Finger{digit}{suffix}'
                left_finger = name.replace(' R ', ' L ')
                closed = profile['closed_finger_local_rotations'][left_finger] @ turn_about((0., 0., 1.), degrees)
                if digit == 0:
                    closed = (closed @ turn_about((0., 1., 0.), thumb_opposition[joint])
                              @ turn_about((1., 0., 0.), 3.))
                grasp.fingers[name][:3, :3] = reflection @ closed @ reflection

    # Balanced martial guard: top of the braced lead hand stays visible gripping
    # the shaft at the lower edge of view without exposing the rigid wrist seam.
    source_pose = Pose(rig, channels, ready_time)
    eye = source_pose.world('Camera')[:3, 3]
    rest = np.array((8.2, 24.0, -10.9))
    aim_point = np.array((1.5, 101., -.6))
    line_of_aim = aim_point - rest
    drive_axis = unit(line_of_aim)
    bind_pose = Pose(rig, {}, 0.)
    bind_wrists = {side: bind_pose.local(grasp.names[-1])[:3, :3] for side, grasp in grasps.items()}
    right_fingers_open = {name: bind_pose.local(name) for name in grasps['R'].fingers}

    def fitted_grip(item, lead_slide=0., r_clav_yaw=0., regrip=0.):
        result = {n: source_pose.local(n) for n in animated}
        for side, grasp in grasps.items():
            clavicle, upper, fore, wrist = grasp.names
            opposite = clavicle.replace(' L ', ' R ') if side == 'L' else clavicle.replace(' R ', ' L ')
            other = source_pose.world(opposite)
            shoulder_base = matrix(mirror_world @ other[:3, :3] @ mirror_local, mirror_world @ other[:3, 3])
            if side == 'R':
                shoulder_base = blend(source_pose.world(clavicle), shoulder_base, .4)
                if abs(r_clav_yaw) > 1e-4:
                    # Pure rotation around clavicle socket: zero socket drift,
                    # natural shoulder retraction on chamber & protraction on lunge.
                    shoulder_base = matrix(turn_about((0., 0., 1.), r_clav_yaw) @ shoulder_base[:3, :3],
                                           shoulder_base[:3, 3])
            result[clavicle] = inverse(source_pose.world(rig.parents[clavicle])) @ shoulder_base
            shoulder = (shoulder_base @ source_pose.local(upper))[:3, 3]
            offset = matrix(np.eye(3), shoulder - grasp.shoulder)
            mount = item.copy()
            if side == 'L':
                mount[:3, 3] += item[:3, 1] * lead_slide
            elif regrip:
                # Pivot about the shaft centre and move the rear hand slightly
                # forward. The lead hand continues to carry the spear.
                mount = item @ matrix(turn_about(shaft_axis, 110. * regrip), shaft_axis * (5. * regrip))
            hand = mount @ grasp.grip
            if side == 'L':
                # Keep the lead elbow naturally low and out of view instead of
                # chicken-winging up into the frame.
                pole = np.array((-.8, -.5, -1.5))
            else:
                outward = np.array((.8, -.7, -1.15))
                pole = 1.75 * outward - hand[:3, :3] @ grasp.palm_along
                if regrip:
                    wrist_axis = unit(hand[:3, 3] - shoulder)
                    pole = turn_about(wrist_axis, 48. * regrip) @ pole
            chain, _ = grasp.fit(inverse(offset) @ mount, elbow_hint=grasp.shoulder + pole, swivels=np.zeros(1))
            worlds = [offset @ joint for joint in chain[1:]]
            pronation = 1. if side == 'L' else .66 * regrip
            if pronation:
                # Share axial rotation with the forearm around its bone axis.
                fore_w, hand_w = worlds[1].copy(), worlds[2]
                rel_x = fore_w[:3, :3].T @ (hand_w[:3, :3] @ bind_wrists[side].T)
                add_tw = np.degrees(np.arctan2(rel_x[2, 1] - rel_x[1, 2], rel_x[1, 1] + rel_x[2, 2]))
                fore_w[:3, :3] = fore_w[:3, :3] @ turn_about((1., 0., 0.), pronation * add_tw)
                worlds[1] = fore_w
            parent = shoulder_base
            for name, world in zip((upper, fore, wrist), worlds):
                result[name] = inverse(parent) @ world
                parent = world
            result.update(grasp.fingers)
            if side == 'R' and 0. < regrip < 1.:
                opening = .42 * np.sin(np.pi * regrip)
                for name, closed in grasp.fingers.items():
                    result[name] = blend(closed, right_fingers_open[name], opening)
        result[ITEM] = inverse(LocalPose(rig, result).world(rig.parents[ITEM])) @ item
        return result

    def spear_pose(anchor_offset, aim_offset=(0., 0.), lead_slide=0., roll_deg=0., r_clav_yaw=0.):
        anchor = eye + rest + np.asarray(anchor_offset, dtype=float)
        target_world = eye + aim_point + np.array((aim_offset[0], 0., aim_offset[1]))
        axis = unit(target_world - anchor)
        rotation = turn_about(axis, roll_deg) @ rotate_between(contact_item[:3, 1], axis) @ contact_item[:3, :3]
        item = matrix(rotation, anchor - lead_station * axis)
        return fitted_grip(item, lead_slide=lead_slide, r_clav_yaw=r_clav_yaw)

    def locked_line_pose(chamber, target, progress, slide_from, slide_to, travel,
                         roll_from, roll_to, clav_from, clav_to, dip_z=0.):
        """Drive the spear strictly straight along the chamber->target strike line."""
        cx, cdraw, cz, _ = chamber
        chamber_anchor = np.array((cx, 0., cz)) - drive_axis * cdraw
        target_world = eye + aim_point + np.array((target[0], 0., target[1]))
        strike_axis = unit(target_world - (eye + rest + chamber_anchor))
        anchor = eye + rest + chamber_anchor + strike_axis * (travel * progress) + np.array((0., 0., dip_z))
        axis = strike_axis
        roll_deg = lerp(roll_from, roll_to, progress)
        rotation = turn_about(axis, roll_deg) @ rotate_between(contact_item[:3, 1], axis) @ contact_item[:3, :3]
        item = matrix(rotation, anchor - lead_station * axis)
        return fitted_grip(item,
                           lead_slide=lerp(slide_from, slide_to, progress),
                           r_clav_yaw=lerp(clav_from, clav_to, progress))

    guard_local = spear_pose((0., 0., 0.))

    def quick_jab(elapsed, name):
        chamber = CHAMBERS[name]
        target = AIM[name]
        cx, cdraw, cz, croll = chamber
        chamber_anchor = np.array((cx, 0., cz)) - drive_axis * cdraw
        # Total forward spear travel from chamber along the locked strike line: ~26.5 units.
        travel = cdraw + 19.0
        drive_end = QUICK_CHAMBER + QUICK_DRIVE
        follow_end = drive_end + QUICK_FOLLOW
        settle_end = drive_end + QUICK_SETTLE

        if elapsed < QUICK_CHAMBER:
            # Snappy micro-chamber: pull back through lead palm and align on strike line.
            w = ease(elapsed / QUICK_CHAMBER)
            return spear_pose(
                chamber_anchor * w,
                aim_offset=(target[0] * w, target[1] * w),
                lead_slide=QUICK_DRAW_SLIDE * w,
                roll_deg=croll * w,
                r_clav_yaw=-6. * w,
            )
        if elapsed < drive_end:
            # Explosive locked-axis drive: accelerates continuously into contact.
            u = (elapsed - QUICK_CHAMBER) / QUICK_DRIVE
            p = QUICK_CONTACT * (u * u * (2.2 - 1.2 * u))
            return locked_line_pose(chamber, target, p, QUICK_DRAW_SLIDE, QUICK_DRIVE_SLIDE,
                                    travel, croll, -0.25 * croll, -6., 10.)
        if elapsed < follow_end:
            # Stiff impact follow-through: brakes rapidly over ~28ms with subtle weight dip.
            u = (elapsed - drive_end) / QUICK_FOLLOW
            rem = 1. - u
            p = 1. - (1. - QUICK_CONTACT) * rem * rem
            dip = -.45 * np.sin(np.pi * u)
            return locked_line_pose(chamber, target, p, QUICK_DRAW_SLIDE, QUICK_DRIVE_SLIDE,
                                    travel, croll, -0.25 * croll, -6., 10., dip_z=dip)
        if elapsed < settle_end:
            # Brief firm recoil rebound (12ms) — no frozen midair stall.
            u = (elapsed - follow_end) / max(1e-5, settle_end - follow_end)
            p = 1. - .05 * ease(u)
            return locked_line_pose(chamber, target, p, QUICK_DRAW_SLIDE, QUICK_DRIVE_SLIDE,
                                    travel, croll, -0.25 * croll, -6., 10.)
        # Crisp elastic recovery back to guard with a subtle lower recovery arc.
        u = min(1., (elapsed - settle_end) / QUICK_RETURN)
        w = ease(u)
        target_world = eye + aim_point + np.array((target[0], 0., target[1]))
        strike_axis = unit(target_world - (eye + rest + chamber_anchor))
        end_anchor = chamber_anchor + strike_axis * (travel * .95)
        arc = np.array((.6, 0., -1.2)) * np.sin(np.pi * w)
        return spear_pose(
            lerp(end_anchor, np.zeros(3), w) + arc,
            aim_offset=(target[0] * (1. - w), target[1] * (1. - w)),
            lead_slide=lerp(lerp(QUICK_DRAW_SLIDE, QUICK_DRIVE_SLIDE, .95), 0., w),
            roll_deg=lerp(-0.25 * croll, 0., w),
            r_clav_yaw=lerp(10., 0., w),
        )

    def charged_chamber_pose(draw, breath=0.):
        cx, cdraw, cz, croll = CHAMBERS['thrust']
        chamber_anchor = np.array((cx, 0., cz)) - drive_axis * cdraw
        # Subtle taut breathing while holding charge (periodic at 0/1).
        b_shift = np.array((.18 * breath, -.45 * breath, .35 * breath))
        return spear_pose(
            chamber_anchor * draw + b_shift,
            aim_offset=(0., -.8 * draw + .25 * breath),
            lead_slide=CHARGE_DRAW_SLIDE * draw + .35 * breath,
            roll_deg=croll * draw + .45 * breath,
            r_clav_yaw=-10. * draw - .6 * breath,
        )

    def charged_jab(elapsed, name):
        chamber = CHAMBERS[name]
        target = AIM[name]
        cx, cdraw, cz, croll = chamber
        chamber_anchor = np.array((cx, 0., cz)) - drive_axis * cdraw
        # Full heavy lunge: ~33.5 units along locked axis through the braced front hand.
        travel = cdraw + 21.5
        follow_end = CHARGED_DRIVE + CHARGED_FOLLOW
        hold_end = CHARGED_DRIVE + JAB_END
        contact_p = .90
        if elapsed < CHARGED_DRIVE:
            # Violent accelerating lunge — peak speed at 'hit', NOT zero!
            u = elapsed / CHARGED_DRIVE
            p = contact_p * (u * u * (2.25 - 1.25 * u))
            return locked_line_pose(chamber, target, p, CHARGE_DRAW_SLIDE, CHARGE_DRIVE_SLIDE,
                                    travel, croll, 3., -10., 14.)
        if elapsed < follow_end:
            # Heavy penetration brake + subtle forward weight dip.
            u = (elapsed - CHARGED_DRIVE) / CHARGED_FOLLOW
            rem = 1. - u
            p = 1. - (1. - contact_p) * rem * rem
            dip = -.85 * np.sin(np.pi * .5 * u)
            return locked_line_pose(chamber, target, p, CHARGE_DRAW_SLIDE, CHARGE_DRIVE_SLIDE,
                                    travel, croll, 3., -10., 14., dip_z=dip)
        if elapsed < hold_end:
            # Brief planted recoil before pulling the spear free.
            u = (elapsed - follow_end) / max(1e-5, hold_end - follow_end)
            p = 1. - .06 * ease(u)
            dip = -.85 * (1. - .4 * ease(u))
            return locked_line_pose(chamber, target, p, CHARGE_DRAW_SLIDE, CHARGE_DRIVE_SLIDE,
                                    travel, croll, 3., -10., 14., dip_z=dip)
        # Decisive withdrawal back to guard.
        u = min(1., (elapsed - hold_end) / CHARGED_RETURN)
        w = ease(u)
        target_world = eye + aim_point + np.array((target[0], 0., target[1]))
        strike_axis = unit(target_world - (eye + rest + chamber_anchor))
        end_anchor = chamber_anchor + strike_axis * (travel * .94) + np.array((0., 0., -.51))
        arc = np.array((1.0, 0., -1.5)) * np.sin(np.pi * w)
        return spear_pose(
            lerp(end_anchor, np.zeros(3), w) + arc,
            aim_offset=(target[0] * (1. - w), target[1] * (1. - w)),
            lead_slide=lerp(lerp(CHARGE_DRAW_SLIDE, CHARGE_DRIVE_SLIDE, .94), 0., w),
            roll_deg=lerp(3., 0., w),
            r_clav_yaw=lerp(14., 0., w),
        )

    cursor = max(float(t) for t, _ in text.keys) + 1.
    hit = QUICK_CHAMBER + QUICK_DRIVE
    length = hit + QUICK_SETTLE + QUICK_RETURN
    events = [(0., 'start'), (QUICK_CHAMBER * .72, 'plant'), (QUICK_CHAMBER, 'swing'),
              (hit, 'thrust hit'), (hit + QUICK_FOLLOW, 'weight forward'),
              (hit + QUICK_SETTLE + .025, 'settle end'), (length, 'stop')]
    for letter in 'abcd':
        for name in (('fasta', 'fastloopa') if letter == 'a' else ('fast' + letter,)):
            section(cursor, length, lambda e, clip=name: quick_jab(e, clip), name, events, rate=CHARGED_SAMPLE_RATE)
            cursor += length + .2
        section(cursor, .05, lambda e: guard_local, 'fastend' + letter, [(0., 'start'), (.05, 'stop')])
        cursor += .25

    # Charged thrust: deep coiled draw with shaft sliding through lead palm,
    # taut breathing hold, and violent accelerating release.
    holds = []
    total = CHARGED_DRIVE + JAB_END + CHARGED_RETURN
    for kind in ('thrust', 'neutral'):
        start, maximum, release = (marks[kind + ' ' + k] for k in ('start', 'max attack', 'min hit'))
        section(start, maximum - start,
                lambda e, s=maximum - start: charged_chamber_pose(ease(e / s) ** 1.15),
                kind, [(0., 'start'), (marks[kind + ' min attack'] - start, 'min attack'), (maximum - start, 'max attack')])
        hold, hold_stop = marks[kind + ' hold start'], marks[kind + ' hold stop']
        hold_len = hold_stop - hold
        holds.append((hold, hold_stop))
        section(hold, hold_len,
                lambda e, hl=hold_len: charged_chamber_pose(1., breath=np.sin(2. * np.pi * (e / max(1e-5, hl)))),
                kind + ' hold',
                [(0., 'start'), (0., 'loop start'), (hold_len * .5, 'breath'),
                 (hold_len, 'loop stop'), (hold_len, 'stop')])
        section(release, total, lambda e, clip=kind: charged_jab(e, clip),
                kind, [(0., 'min hit'), (0., 'plant'), (CHARGED_DRIVE, 'hit'),
                       (CHARGED_DRIVE + CHARGED_FOLLOW, 'weight forward'),
                       (CHARGED_DRIVE + JAB_END + .04, 'settle end'),
                       (CHARGED_DRIVE + JAB_END + .08, 'chain')] +
                [(t, size + ' follow ' + edge) for t, edge in ((CHARGED_DRIVE + .001, 'start'), (total, 'stop'))
                 for size in ('small', 'medium', 'large')], rate=CHARGED_SAMPLE_RATE)

    # Moving between the spear guard and a staff pose: DM regrips the arms
    # (upper arms differ by ~170 degrees of twist), so blending the bones
    # spins the hands and the spear in them. The spear moves on its own
    # world path instead; both palms follow it on their grips and the arms
    # reach them.
    class Mix:
        """Blend two moving poses without flips. The rotation between a staff
        pose and the guard can be near 180 degrees, so the shortest way round
        changes side from one sample to the next. Each channel keeps the side
        it starts on for the whole transition."""
        def __init__(self, held=True, dip=DIP, follow_elbows=False):
            self.signs = {}
            self.held = held
            self.dip = np.array(dip)
            self.follow_elbows = follow_elbows
            self.elbow_turns = {}

        def turn(self, key, a, b, w):
            qa, qb = matrix_quat(a[:3, :3]), matrix_quat(b[:3, :3])
            last = self.signs.get(key)
            if last is None:
                qb = qb if qa @ qb >= 0 else -qb
            else:
                qa = qa if qa @ last[0] >= 0 else -qa
                qb = qb if qb @ last[1] >= 0 else -qb
            self.signs[key] = (qa, qb)
            angle = np.arccos(np.clip(qa @ qb, -1., 1.))
            if angle < 1e-6:
                q = qa
            else:
                q = (np.sin((1 - w) * angle) * qa + np.sin(w * angle) * qb) / np.sin(angle)
            return matrix(quat_matrix(q[[1, 2, 3, 0]]), a[:3, 3] + w * (b[:3, 3] - a[:3, 3]))

        def arm(self, side, start, end, base, grasp, hand, weight, parent):
            clavicle, upper, fore, wrist = grasp.names
            clavicle_world = base.world(clavicle).copy()
            shoulder = base.world(upper)[:3, 3].copy()
            delta = hand[:3, 3] - shoulder
            distance = np.linalg.norm(delta)
            axis = delta / distance
            upper_length, fore_length = grasp.segments
            reach = np.clip(distance, abs(upper_length - fore_length) + .35, upper_length + fore_length - .35)
            excess = distance - reach
            shoulder += axis * excess
            clavicle_world[:3, 3] += axis * excess

            def arm_frame(pose):
                u, f, h = (pose.world(n) for n in (upper, fore, wrist))
                first, second = f[:3, 3] - u[:3, 3], h[:3, 3] - f[:3, 3]
                line = unit(first + second)
                pole = unit(first - line * (line @ first))
                normal = np.cross(first, second)
                first_offset = basis(first, normal).T @ u[:3, :3]
                second_offset = basis(second, normal).T @ f[:3, :3]
                return rotate_between(line, axis) @ pole, first_offset, second_offset

            from_pole, from_upper, from_fore = arm_frame(start)
            to_pole, to_upper, to_fore = arm_frame(end)
            # Blend around the arm's bend circle. Interpolating two elbow
            # positions can pass through the wrist axis and flip the upper arm.
            turn = np.degrees(np.arctan2(axis @ np.cross(from_pole, to_pole), from_pole @ to_pole))
            if side in self.elbow_turns:
                turn += 360. * round((self.elbow_turns[side] - turn) / 360.)
            self.elbow_turns[side] = turn
            pole = turn_about(axis, weight * turn) @ from_pole
            along = (upper_length**2 - fore_length**2 + reach**2) / (2. * reach)
            elbow = shoulder + axis * along + pole * np.sqrt(max(0., upper_length**2 - along**2))
            first, second = elbow - shoulder, hand[:3, 3] - elbow
            normal = np.cross(first, second)

            def joint(name, direction, origin, frame_a, frame_b):
                offset = self.turn(name, matrix(frame_a, (0., 0., 0.)), matrix(frame_b, (0., 0., 0.)), weight)
                return matrix(basis(direction, normal) @ offset[:3, :3], origin)

            upper_world = joint(upper + ' bend', first, shoulder, from_upper, to_upper)
            fore_world = joint(fore + ' bend', second, elbow, from_fore, to_fore)
            return {clavicle: inverse(parent) @ clavicle_world, upper: inverse(clavicle_world) @ upper_world,
                    fore: inverse(upper_world) @ fore_world, wrist: inverse(fore_world) @ hand}

        def __call__(self, a, b, w):
            w = min(1., max(0., w))
            if self.follow_elbows and w in (0., 1.):
                return a if w == 0. else b
            out = {n: self.turn(n, a[n], b[n], w) for n in animated}
            if not self.held:
                return out
            A, B = LocalPose(rig, a), LocalPose(rig, b)
            spear = self.turn('spear', A.world(ITEM), B.world(ITEM), w)
            # The guard's hands sit beside the head; on the way to a staff
            # pose they pass under the eye instead of across it.
            spear[:3, 3] += self.dip * np.sin(np.pi * w)
            now = LocalPose(rig, out)
            for side, grasp in grasps.items():
                hand = grasp.names[-1]
                grip = self.turn('grip ' + side, inverse(A.world(ITEM)) @ A.world(hand),
                                 inverse(B.world(ITEM)) @ B.world(hand), w)
                neck = now.world(rig.parents[grasp.names[0]])
                if self.follow_elbows:
                    out.update(self.arm(side, A, B, now, grasp, spear @ grip, w, neck))
                else:
                    out.update(steer_arm(now, now, grasp, spear @ grip, 0., parent=neck))
                now = LocalPose(rig, out)
            out[ITEM] = inverse(now.world(rig.parents[ITEM])) @ spear
            return out

    # Side swing: from the guard straight into DM's wind-up, which keeps
    # running (slower, to the same max attack key); leave the recovery when
    # the point faces forward again and settle into the guard. The follow
    # stops move to the end of that settle.
    swing, wound = marks['slash start'], marks['slash max attack']
    joined = swing + SWING_JOIN
    section(swing, wound - swing,
            lambda e, mix=Mix(follow_elbows=True):
                mix(guard_local, at(joined + e / (wound - swing) * (wound - joined)), ease(e / JOIN)),
            'slash', [(0., 'start'), (marks['slash min attack'] - swing, 'min attack'),
                      (wound - swing, 'max attack')])
    leave = marks['slash hit2'] + SWING_LEAVE
    section(leave, LEAVE, lambda e, mix=Mix(dip=SWEEP_DIP, follow_elbows=True):
                mix(at(leave + recovery_clock(e, LEAVE)), guard_local, ease(e / LEAVE)),
            'slash', [(LEAVE, size + ' follow stop') for size in ('small', 'medium', 'large')])

    for name, (kind, marker, lead, follow) in SWEEPS.items():
        contact = marks[marker]
        start, stop = contact - lead, contact + follow
        recovery = SWEEP_ENTRY + stop - start
        duration = recovery + SWEEP_RETURN

        def sweep(e, a=start, b=stop, mix_in=Mix(dip=SWEEP_DIP, follow_elbows=True),
                  mix_out=Mix(dip=SWEEP_DIP, follow_elbows=True)):
            time, weight = sweep_clock(e, a, b)
            if weight == 0.:
                return guard_local
            authored = at(time)
            if weight == 1.:
                return authored
            return (mix_in if e < SWEEP_ENTRY else mix_out)(guard_local, authored, weight)

        section(cursor, duration, sweep, name,
                [(0., 'start'), (SWEEP_ENTRY, 'swing'), (SWEEP_ENTRY + lead, kind + ' hit'),
                 (recovery, 'recovery'), (duration, 'stop')])
        cursor += duration + .2

    # Draw and sheathe: DM's draw brings the haft over the shoulder into the
    # staff guard; the spear leaves that path for its own guard while it is
    # still coming down. Sheathing lets go of the spear at once (detach key),
    # so only the arms blend back into DM's sheathe.
    drawn_at, drawn_end = marks['equip attach'] - DRAW_EARLY, marks['equip stop']
    section(drawn_at, drawn_end - drawn_at,
            lambda e, mix=Mix(): mix(at(drawn_at + e), guard_local, ease(e / (drawn_end - drawn_at))), 'equip', [])
    sheathe, sheathe_to = marks['unequip start'], marks['unequip detach'] + .25
    section(sheathe, sheathe_to - sheathe,
            lambda e, mix=Mix(held=False): mix(guard_local, at(sheathe + e), ease(e / (sheathe_to - sheathe))),
            'unequip', [])

    # Sample and merge: replace keys inside every authored interval.
    replace_frames(stream, sampled_poses(segments), [(begin, stop) for begin, stop, _, _ in segments], cursor)

    replaced = ('fasta ', 'fastb ', 'fastc ', 'fastd ', 'fastend', 'fastloopa ', 'thrust ', 'neutral ')
    kept = []
    for t, label in text.keys:
        lines = [l for l in label.splitlines()
                 if not l.removeprefix('weapononehand: ').startswith(replaced)
                 and l.removeprefix('weapononehand: ') not in (
                     'slash start', 'slash min attack', 'slash max attack',
                     'slash small follow stop', 'slash medium follow stop', 'slash large follow stop')
                 and not (any(h <= t <= e for h, e in holds)
                          and l in ('weapononehand: loop start', 'weapononehand: loop stop'))]
        if lines:
            kept.append((float(t), '\r\n'.join(lines)))
    kept += labels + [(t, 'weapononehand: loop ' + edge) for h, e in holds for t, edge in ((h, 'start'), (e, 'stop'))]
    kept.append((0., 'weapononehand: brutal spear grip'))
    kept.sort(key=lambda p: p[0])
    text.keys = np.array(kept, dtype=text.keys.dtype)
    stream.save(target / source.name)
    # Strokes start where the previous one ended: keep only a short blend.
    rules = source.with_suffix('.yaml').read_text()
    rules = re.sub(r"(to: 'weapononehand:fast(?:end)?[abcd] start'\s+easing: linear\s+duration: )0\.2[0-9]*",
                   r"\g<1>0.045", rules)
    for name in ('fastloopa', *SWEEPS):
        rules += f"  - from: '*'\n    to: 'weapononehand:{name} start'\n    easing: linear\n    duration: 0.045\n"
    (target / 'weapononehand.yaml').write_text(rules)

    # The rear hand regrips during the raise while the lead hand guides the
    # spear. Reverse the same path on lower, including after an interrupted raise.
    block = folder / 'brutal_block.kf'
    block_marks = native_starts(block)
    start, hold, lower, end = (block_marks[f'brutal_block: {key}']
                                for key in ('start', 'loop start', 'lower start', 'stop'))
    guard_rotation = LocalPose(rig, guard_local).world(ITEM)[:3, :3]

    def block_pose_at(phase):
        w = ease(phase)
        yaw = 52. * w
        pitch = 14. * w
        roll = -20. * w
        rot = (turn_about((0., 0., 1.), yaw) @ turn_about((1., 0., 0.), pitch)
               @ turn_about(drive_axis, roll) @ guard_rotation)
        # Smooth upward-left defensive parry bar across the view; a subtle
        # outward arc keeps the shaft clear of the camera mid-raise.
        anchor_off = np.array((-10.5, 3.0, 7.5)) * w + np.array((0., 2.0, -1.5)) * np.sin(np.pi * w)
        item = matrix(rot, eye + rest + anchor_off - lead_station * rot[:, 1])
        regrip = ease((phase - .07) / .83)
        return fitted_grip(item, lead_slide=5.5 * w, r_clav_yaw=4. * w, regrip=regrip)

    phases = np.linspace(0., 1., int(np.ceil(max(hold - start, end - lower) * (2 * RATE))) + 1)
    poses = [guard_local, *(block_pose_at(p) for p in phases[1:])]
    block_samples = dict(zip(lerp(start, hold, phases), poses))
    # Keep the completed regrip through the held loop.
    hold_pose = poses[-1]
    for t in np.linspace(hold, lower, max(2, int(np.ceil((lower - hold) * 15.)) + 1)):
        block_samples[float(t)] = hold_pose
    block_samples.update(zip(lerp(lower, end, phases), reversed(poses)))
    block_stream = load(block)
    replace_frames(block_stream, block_samples, [(start, end)])
    block_stream.save(target / block.name)
    shutil.copyfile(block.with_suffix('.yaml'), target / block.with_suffix('.yaml').name)

    def gait_pose(phase, mode):
        """Rigid, braced spear carry locked to the stride (no floppy tip whip).

        - walk:   subtle grounded step carry so the spear isn't a frozen statue.
        - run:    athletic braced guard with synced translational stride bounce
                  and tight angular discipline (<= ~1.8 deg).
        - sprint: couched battering-ram carry tucked firmly at the right side,
                  tip aimed steadily forward-low below the crosshair with heavy
                  linear drive and minimal angular wobble.
        """
        stride = 2. * np.pi * phase
        plant = 2. * stride - 1.16 * np.pi
        if mode == 'sprint':
            # Couched ram carry: dropped & tucked firmly into right ribs/hip,
            # rigid forward-low point, heavy rhythmic pump synced to footfalls.
            base_shift = np.array((2.4, -3.2, -3.4))
            travel = base_shift + np.array((
                .85 * np.sin(stride),
                1.35 * np.cos(plant - .25),
                -1.45 * np.cos(plant),
            ))
            yaw = -1.4 + .75 * np.sin(stride)
            pitch = -3.4 - .95 * np.cos(plant)
            roll = -7.5 + 1.1 * np.sin(stride)
            lead_slide = 1.8
            r_clav = -4.0
        elif mode == 'run':
            # Locked combat run: translational stride bounce, very tight angular hold.
            base_shift = np.array((.6, -.6, -1.1))
            travel = base_shift + np.array((
                1.05 * np.sin(stride),
                .75 * np.cos(plant - .2),
                -1.25 * np.cos(plant),
            ))
            yaw = .75 * np.sin(stride)
            pitch = -.6 - .85 * np.cos(plant)
            roll = 1.0 * np.sin(stride)
            lead_slide = .5
            r_clav = -1.0
        else:
            # Subtle, grounded walking rhythm.
            travel = np.array((
                .40 * np.sin(stride),
                .25 * np.cos(plant),
                -.45 * np.cos(plant),
            ))
            yaw = .30 * np.sin(stride)
            pitch = -.30 * np.cos(plant)
            roll = .40 * np.sin(stride)
            lead_slide = 0.
            r_clav = 0.
        rotation = (turn_about((0., 0., 1.), yaw) @ turn_about((1., 0., 0.), pitch)
                    @ turn_about(drive_axis, roll) @ guard_rotation)
        item = matrix(rotation, eye + rest + travel - lead_station * rotation[:, 1])
        return fitted_grip(item, lead_slide=lead_slide, r_clav_yaw=r_clav)

    # Keep the pose of both palms relative to the shaft through the gait.
    # The original trunk and camera tracks retain their motion.
    arm_bones = [name for grasp in grasps.values() for name in (*grasp.names, *grasp.fingers)] + [ITEM]
    for path in sorted(folder.glob('*1h.kf')):
        if not path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            continue
        gait = load(path)
        sprint = path.stem.startswith('brutal_sprint')
        run = path.stem.startswith('run')
        walk = path.stem.startswith('walk')
        if sprint or run or walk:
            mode = 'sprint' if sprint else ('run' if run else 'walk')
            start_key, stop_key = (path.stem + ': ' + key for key in ('loop start', 'loop stop'))
            source_marks = native_starts(path)
            begin, end = source_marks[start_key], source_marks[stop_key]
            body_gait = body / 'Animations/brutal_firstperson' / path.name if body is not None else None
            if body_gait is not None and body_gait.is_file():
                body_marks = native_starts(body_gait)
                ratio = (body_marks[stop_key] - body_marks[start_key]) / (end - begin)
                if abs(ratio - 1.) > 1e-5:
                    retime_gait(gait, ratio)
                    begin, end = begin * ratio, end * ratio
            times = np.linspace(begin, end, int(np.ceil((end - begin) * RATE)) + 1)
            poses = {}
            for time in times:
                pose = gait_pose((time - begin) / (end - begin), mode)
                poses[float(time)] = {name: pose[name] for name in arm_bones if name in pose}
            replace_frames(gait, poses, [(begin, end)])
        else:
            gait_names = [e.string_data for e in gait.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
            for name, controller in zip(gait_names, gait.root.controllers):
                if name not in arm_bones or name not in guard_local:
                    continue
                for index, field in enumerate((controller.data.rotations, controller.data.translations)):
                    old = field.keys
                    if old is None or not len(old):
                        continue
                    value = matrix_quat(guard_local[name][:3, :3]) if index == 0 else guard_local[name][:3, 3]
                    field.keys = np.column_stack((old[:, 0], np.tile(value, (len(old), 1)))).astype(np.float32)
        gait.save(target / path.name)
        if path.with_suffix('.yaml').exists():
            shutil.copyfile(path.with_suffix('.yaml'), target / path.with_suffix('.yaml').name)
    if body is not None:
        body_footwork(body, target / source.name)
    manifest_path = arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())
    for path in sorted(target.iterdir()):
        manifest['outputs'][path.relative_to(arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['spear'] = {'base': 'brutal_fp_staff', 'jab': 'dm staff charged attack, second jab',
                         'ready_before_hit': READY_BEFORE_HIT, 'quick_hit': hit, 'quick_stop': length,
                         'aim': AIM, 'chambers': CHAMBERS}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arms', type=Path, required=True)
    parser.add_argument('--body', type=Path)
    args = parser.parse_args()
    build(args.arms, args.body)
