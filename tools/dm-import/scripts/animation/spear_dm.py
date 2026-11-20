#!/usr/bin/env python3
"""Spear strokes made of Dark Messiah's one straight staff jab.

Every DM staff stroke that is labelled a thrust starts with an overhead swing,
except one: the second jab of the charged staff attack. There the haft keeps
pointing forward, the rear fist draws it back while the left hand regrips,
then drives it forward to the crosshair and holds. The spear's guard is
that chamber. Quick thrusts drive from it and return to it. A charged
thrust pulls the haft farther back through the front hand before driving.
Sidesteps use the staff's swings. The staff's parry, draw and sheathe
join the spear guard along a continuous path for the haft and both hands.
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
JAB_CHAMBER, JAB_END = -.19, .18   # deepest draw (both hands on the haft), end of the steady hold
SETTLE = .12          # each stroke holds the chamber while the engine blends into it
RETURN = .38          # draw back from the held thrust to the guard
RATE = 60.
CHARGE_DRAW = np.array((8., -15., -7.))
CHARGE_SLIDE = 19.      # the front palm lets the shaft slide as the rear palm pulls
CHARGE_RELEASE = .16   # time to regain the donor's drive after the extra draw
SWEEPS = {
    # Use the two passes across the front in the staff's charged slash. The
    # quick staff combo twirls the haft behind the player between attacks.
    'faststrafeleft': ('slashright', 'slash hit2', .136, .060),
    'faststraferight': ('slash', 'slash hit', .106, .070),
}
SWEEP_ENTRY, SWEEP_RETURN = .35, .46
SWEEP_LEAD, SWEEP_TAIL = .065, .112
SWEEP_DIP = (0., 1., -5.)            # clear the view without folding the rear arm against its shoulder
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
# A person never strikes the same spot twice. Each quick stroke turns both
# arms and the spear together about the neck by its own (yaw, pitch, roll) in
# degrees: z up, positive yaw to the left, positive pitch tip up, positive roll
# clockwise from behind. The turn grows during the drive and goes back during
# the draw, so every stroke still starts and ends in the same guard.
AIM = {'fasta': (5.5, -2.2, -8.), 'fastb': (-6., 1.8, 9.), 'fastc': (1.6, -5.2, 4.),
       'fastd': (-3.6, -3., -10.), 'fastloopa': (6.4, 1., 6.), 'thrust': (0., -.8, 0.),
       'neutral': (-1.5, -1.5, -3.)}


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
    for begin, stop, fn in segments:
        for t in np.linspace(begin, stop, max(2, int(np.ceil((stop - begin) * RATE)) + 1)):
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
        start, plant, hit, stop = [marks[clip+' '+key] for key in ('start','plant','thrust hit','stop')]
        lead = .5 if clip in ('fastb', 'fastd') else 0.
        ranges[clip] = [(start, lead-.13, 0.), (plant, lead-.05, .36),
                        (hit, lead+.22, .76), (hit+.085, lead+.29, .76), (stop, lead+.48, 0.)]
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
    chamber, jab1 = hit2 + JAB_CHAMBER, hit2 + JAB_END
    # The spear's guard is the jab's drawn chamber: haft forward, both hands
    # on it. Thrusts start and end there without sweeping across the view.

    def at(t, source_channels=channels):
        pose = Pose(rig, source_channels, t)
        return {n: pose.local(n) for n in animated}

    guard_local = at(chamber)

    segments = []          # (start, stop, fn(elapsed) -> local transforms)
    labels = []

    def section(start, length, fn, name, events):
        segments.append((start, start + length, fn))
        labels.extend((start + t, 'weapononehand: ' + name + ' ' + e) for t, e in events)

    drive = jab1 - chamber

    reach = hit2 - chamber      # the drive itself, without the static hold

    contact_pose, guard_pose = Pose(rig, channels, hit2), Pose(rig, channels, chamber)
    grasps = {side: Grasp(contact_pose, side, False) for side in 'RL'}
    left = grasps['L']
    grip_contact = inverse(contact_pose.world(ITEM)) @ contact_pose.world(left.names[-1])
    grip_guard = inverse(guard_pose.world(ITEM)) @ guard_pose.world(left.names[-1])

    def returning(e):
        # Draw the haft back along the same line: DM's own drive, reversed,
        # from the contact pose. The static hold is not replayed; its last
        # 0.6-unit settle is closed in the first 0.04 s. The left hand does
        # not replay DM's regrip backwards: it slides smoothly from its
        # contact pose to its guard pose, holding the haft.
        if e < .04:
            return at(jab1 - (jab1 - hit2) * e / .04)
        u = ease(min(1., (e - .04) / (RETURN - .04)))
        pose = at(hit2 - reach * u)
        now = LocalPose(rig, pose)
        # The left palm stays on the moving haft; its station and roll on
        # the haft move from the contact grip to the guard grip.
        hand = now.world(ITEM) @ blend(grip_contact, grip_guard, u)
        neck = now.world(rig.parents[left.names[0]])
        pose.update(steer_arm(contact_pose, guard_pose, left, hand, u, parent=neck))
        for n in left.fingers:
            if n in pose:
                pose[n] = blend(contact_pose.local(n), guard_pose.local(n), u)
        return pose

    clavicles = [n for n in animated if n.endswith('Clavicle')]

    def aimed(pose, angles, weight):
        # Turn both clavicles by one rigid motion about the neck: the arms,
        # hands and spear move as one body and the shoulders stay seated.
        if not weight or not any(angles):
            return pose
        yaw, pitch, roll = (a * weight for a in angles)
        turn = turn_about((0., 0., 1.), yaw) @ turn_about((1., 0., 0.), pitch) @ turn_about((0., 1., 0.), roll)
        pose = dict(pose)
        now = LocalPose(rig, pose)
        for name in clavicles:
            parent = now.world(rig.parents[name])
            pivot = now.world(rig.parents[name])[:3, 3]
            motion = matrix(turn, pivot - turn @ pivot)
            pose[name] = inverse(parent) @ motion @ parent @ pose[name]
        return pose

    def drawn(e):
        # Weight of the stroke's own aim while drawing back.
        return 1. - ease(min(1., max(0., (e - .04) / (RETURN - .04))))

    def draw_haft(pose, weight):
        if not weight:
            return pose
        old = LocalPose(rig, pose)
        base = old.world(ITEM)
        turn = turn_about((0., 0., 1.), -5. * weight) @ turn_about((1., 0., 0.), -3. * weight)
        spear = matrix(turn @ base[:3, :3], base[:3, 3] + weight * CHARGE_DRAW)
        pose = dict(pose)
        for side, grasp in grasps.items():
            grip = inverse(base) @ old.world(grasp.names[-1])
            if side == 'L':
                grip[1, 3] += CHARGE_SLIDE * weight
            pose.update(steer_arm(old, old, grasp, spear @ grip, 0.))
        pose[ITEM] = inverse(LocalPose(rig, pose).world(rig.parents[ITEM])) @ spear
        return pose

    def jab_for(angles, charged=False):
        # Each stroke starts and ends in the guard (the drawn chamber), so a
        # series chains without the engine blending between far-apart poses:
        # settle, DM's drive and held extension, then draw back to guard.
        def jab(elapsed):
            if elapsed <= SETTLE:
                return draw_haft(guard_local, 1.) if charged else guard_local
            if elapsed <= SETTLE + drive:
                drive_time = elapsed - SETTLE
                pose = at(chamber + drive_time)
                if charged:
                    pose = draw_haft(pose, 1. - ease(drive_time / CHARGE_RELEASE))
                return aimed(pose, angles, ease(drive_time / reach))
            e = elapsed - SETTLE - drive
            return aimed(returning(e), angles, drawn(e))
        return jab

    cursor = max(float(t) for t, _ in text.keys) + 1.
    hit = SETTLE + hit2 - chamber
    length = SETTLE + drive + RETURN
    events = [(0., 'start'), (SETTLE - .04, 'plant'), (SETTLE, 'swing'),
              (hit, 'thrust hit'), (hit + .035, 'weight forward'), (hit + .07, 'settle end'), (length, 'stop')]
    for letter in 'abcd':
        for name in (('fasta', 'fastloopa') if letter == 'a' else ('fast' + letter,)):
            section(cursor, length, jab_for(AIM[name]), name, events)
            cursor += length + .2
        section(cursor, .05, lambda e: guard_local, 'fastend' + letter, [(0., 'start'), (.05, 'stop')])
        cursor += .25

    # Charged thrust: draw into the jab's chamber and hold it; release drives
    # it. The standing charge ("neutral") gets the same thrust: the engine
    # starts it on every click before it knows the click is a quick stroke,
    # and the staff's overhead start showed as a jump of the spear.
    chamber_local = draw_haft(guard_local, 1.)
    holds = []
    total = drive + RETURN + .25
    for kind in ('thrust', 'neutral'):
        start, maximum, release = (marks[kind + ' ' + k] for k in ('start', 'max attack', 'min hit'))
        section(start, maximum - start,
                lambda e, s=maximum - start: draw_haft(guard_local, ease(e / s)),
                kind, [(0., 'start'), (marks[kind + ' min attack'] - start, 'min attack'), (maximum - start, 'max attack')])
        hold, hold_stop = marks[kind + ' hold start'], marks[kind + ' hold stop']
        holds.append((hold, hold_stop))
        section(hold, hold_stop - hold, lambda e: chamber_local, kind + ' hold',
                [(0., 'start'), (0., 'loop start'), (hold_stop - hold, 'loop stop'), (hold_stop - hold, 'stop')])
        jab = jab_for(AIM[kind], charged=True)
        section(release, total, lambda e, jab=jab: jab(e + SETTLE),
                kind, [(0., 'min hit'), (0., 'plant'), (hit2 - chamber, 'hit'),
                       (hit2 - chamber + .035, 'weight forward'), (hit2 - chamber + .07, 'settle end'),
                       (drive + .1, 'chain')] +
                [(t, size + ' follow ' + edge) for t, edge in ((hit2 - chamber + .001, 'start'), (total, 'stop'))
                 for size in ('small', 'medium', 'large')])

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
    replace_frames(stream, sampled_poses(segments), [(begin, stop) for begin, stop, _ in segments], cursor)

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

    # A partial raise may be reversed before the block is fully posed. Use the
    # same frames in both directions, so matching the raise/lower phases in
    # CharacterController also matches the actual pose of both hands.
    block = folder / 'brutal_block.kf'
    block_channels = tracks(block)
    block_marks = native_starts(block)
    start, hold, lower, end = (block_marks[f'brutal_block: {key}']
                                for key in ('start', 'loop start', 'lower start', 'stop'))
    block_pose = at(hold, block_channels)
    parry_mix = Mix(dip=(0., 2., -9.))
    phases = np.linspace(0., 1., int(np.ceil(max(hold - start, end - lower) * RATE)) + 1)
    poses = [guard_local, *(parry_mix(guard_local, block_pose, ease(p)) for p in phases[1:-1]), block_pose]
    block_samples = dict(zip(lerp(start, hold, phases), poses))
    block_samples.update(zip(lerp(lower, end, phases), reversed(poses)))
    block_stream = load(block)
    replace_frames(block_stream, block_samples, [(start, hold), (lower, end)])
    block_stream.save(target / block.name)
    shutil.copyfile(block.with_suffix('.yaml'), target / block.with_suffix('.yaml').name)

    # Idle and gait: hold the spear guard in the arms and on the weapon; the
    # gait's own trunk, head and camera motion stay.
    arm_bones = [n for n in animated if n == ITEM or ((' L ' in n or ' R ' in n) and not n.endswith(('Thigh', 'Calf', 'Foot', 'Toe0')))]
    for path in sorted(folder.glob('*1h.kf')):
        if not path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            continue
        gait = load(path)
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
                         'chamber': JAB_CHAMBER, 'end': JAB_END, 'settle': SETTLE, 'return': RETURN, 'aim': AIM}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arms', type=Path, required=True)
    parser.add_argument('--body', type=Path)
    args = parser.parse_args()
    build(args.arms, args.body)
