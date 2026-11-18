#!/usr/bin/env python3
"""First-person spear thrusts, using native finger curls and the DM staff's release/regrip."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from import_staff_native_grasp import swing
from import_two_hand_viewmodel import (
    Grasp, ITEM, Pose, Rig, SIDES, bone, inverse, keys, lerp, load, matrix,
    matrix_quat, native_starts, nif, quat_matrix, slerp, tracks, turn_about, unit,
)


def ease(x):
    x = float(np.clip(x, 0., 1.))
    return x * x * (3. - 2. * x)


def blend(a, b, weight):
    q = slerp(matrix_quat(a[:3, :3]), matrix_quat(b[:3, :3]), weight)
    return matrix(quat_matrix(q[[1, 2, 3, 0]]), lerp(a[:3, 3], b[:3, 3], weight))


def shaft(axis, roll_guide, position):
    y = unit(np.asarray(axis, float))
    x = unit(roll_guide - y * np.dot(y, roll_guide))
    return matrix(np.column_stack((x, y, np.cross(x, y))), position)


class LocalPose(Pose):
    def __init__(self, rig, transforms):
        super().__init__(rig, {}, 0.)
        self.transforms = transforms

    def local(self, name):
        return self.transforms[name] if name in self.transforms else super().local(name)


def relaxed_wrist(forearm, hand, grasp, amount):
    neutral = forearm[:3, :3] @ grasp.wrist_frame
    direction = hand[:3, 3] - forearm[:3, 3]
    natural = swing(neutral @ grasp.palm_along, direction) @ neutral
    return blend(hand, matrix(natural, hand[:3, 3]), amount)


def seat_wrist(arm, grasp, weight):
    """Keep the native wrist joint seated by following its reach with the shoulder."""
    clavicle, upper, fore, hand = [m.copy() for m in arm]
    neutral = matrix(hand[:3, :3] @ grasp.wrist_frame.T, fore[:3, 3])
    aligned = blend(fore, neutral, weight)
    elbow = hand[:3, 3] - aligned[:3, 0] * grasp.segments[1]
    shoulder = elbow + unit(upper[:3, 3] - elbow) * grasp.segments[0]
    clavicle[:3, 3] += shoulder - upper[:3, 3]
    upper[:3, :3] = swing(upper[:3, 0], elbow - shoulder) @ upper[:3, :3]
    upper[:3, 3], aligned[:3, 3] = shoulder, elbow
    return clavicle, upper, aligned, hand


def steer_arm(start, end, grasp, hand, w, relax=0., parent=None):
    """Move a bent arm between two poses, meeting the hand target at both bone lengths."""
    clavicle, upper, fore, wrist = grasp.names
    C, U, F, H = [blend(start.world(n), end.world(n), w) for n in grasp.names]
    a, b = grasp.segments
    delta = hand[:3, 3] - U[:3, 3]
    d = np.linalg.norm(delta)
    axis = unit(delta)
    shifted = axis * max(0., d - (a + b - .35))
    C[:3, 3] += shifted
    U[:3, 3] += shifted
    d -= np.linalg.norm(shifted)
    x = (a*a - b*b + d*d) / (2*d)
    pole = F[:3, 3] - U[:3, 3]
    elbow = U[:3, 3] + axis*x + unit(pole - axis*(axis @ pole))*np.sqrt(max(0., a*a - x*x))
    upper_axis = unit(start.local(fore)[:3, 3])
    fore_axis = unit(start.local(wrist)[:3, 3])
    U[:3, :3] = swing(U[:3, :3] @ upper_axis, elbow - U[:3, 3]) @ U[:3, :3]
    F[:3, :3] = swing(F[:3, :3] @ fore_axis, hand[:3, 3] - elbow) @ F[:3, :3]
    F[:3, 3] = elbow
    if relax:
        hand = relaxed_wrist(F, hand, grasp, relax)
    if parent is None:
        parent = start.world(start.rig.parents[clavicle])
    return {clavicle: inverse(parent) @ C, upper: inverse(C) @ U,
            fore: inverse(U) @ F, wrist: inverse(F) @ hand}


def close_hand(grasp, profile, side):
    # Keep the native palm attitude. Fit its grasp axis and centre to the
    # skin-calibrated power grip used on the same Morrowind shaft meshes.
    mirror = np.diag((1., 1., -1. if side == 'R' else 1.))
    aligned = grasp.grip[:3, :3] @ mirror @ profile['shaft_axis_native_hand_reference']
    y = np.array((0., np.sign(aligned[1]), 0.))
    grasp.grip[:3, :3] = swing(aligned, y) @ grasp.grip[:3, :3]
    centre = grasp.grip[:3, :3] @ mirror @ profile['shaft_anchor_native_hand']
    grasp.grip[[0, 2], 3] = -centre[[0, 2]]
    for name, rotation in profile['closed_finger_local_rotations'].items():
        name = name.replace(' L ', ' ' + side + ' ')
        grasp.fingers[name] = matrix(mirror @ rotation @ mirror, grasp.fingers[name][:3, 3])


class Spear:
    def __init__(self, rig, source, marks, native, fist, profile):
        self.rig = rig
        self.channels = source
        self.release = marks['thrust min hit']
        self.idle = Pose(rig, source, marks['thrust start'])
        self.camera = self.idle.world('Camera')
        self.grasps = [Grasp(native, side, False) for side in SIDES]
        # Keep the staff's right mounting point so it stays in the hand
        # throughout the change of stance. Bring the native supporting palm
        # forward along the shaft, preserving its finger axes and curl.
        right, left = self.grasps
        right.grip = inverse(self.idle.local(ITEM))
        left.grip[1, 3] = right.grip[1, 3] + 29.
        left.pole = unit(np.array((-.8, -.58, .15)))
        left.wrist_bias = 6.
        left.grip = matrix(turn_about((0, 1, 0), -24.), np.zeros(3)) @ left.grip
        for side, g in zip(SIDES, self.grasps):
            close_hand(g, profile, side)
            for name, pose in g.fingers.items():
                weight = .32 if 'Finger0' in name else .27
                g.fingers[name] = blend(pose, fist.local(name), weight)
        self.names = [*source]
        self.open_hand = Pose(rig, source, self.release + 49. / 33.)
        self.guard = self.strike(0., 0., 0., 0.)

    def strike(self, travel, push, across, height, release=0.):
        # Author the screen silhouette in camera space. During a full lunge the
        # rear hand carries the pole while the other hand opens and falls back.
        freed = ease(release)
        diagonal = np.array((-.777, .562, -.284))
        forward = unit(np.array((-9. - across*.70, 4. - height*.7 - 7.*freed, -49.)))
        axis = unit(lerp(diagonal, forward, push) + np.array((-.012*across, -.010*height, 0.))*(1.-push))
        rear = np.array((17.4 + across - 4.*push, -20.3 + height + 6.*push + 16.*freed,
                         -23.4 + 12.*push)) + travel * axis
        item = shaft(axis, np.array((-.78, -.62, 0.)), rear)
        item[:3, 3] -= axis * (self.grasps[0].grip[1, 3] + 1.7)
        item = self.camera @ item
        # The left shoulder leads in the stance. On extension the right
        # shoulder comes through while the support hand lets the shaft travel.
        offsets = [(-.8+3.*freed, -.4+4.*freed, 2.5 - 9.*push - 10.*freed),
                   (2., .2, -7. - 4.*push)]
        reaches = [self.camera[:3, :3] @ shift for shift in offsets]
        stations = [4.*(1.-push), -6.5 * push]
        result = {name: self.idle.local(name) for name in self.names}
        # The spear turns in the carrying palm during a long thrust. Its
        # flat blade banks with the right arm; the two contact points remain
        # centred on the haft as the left hand releases.
        bank = matrix(turn_about((0, 1, 0), -52. + 85.*push + 22.*freed), np.zeros(3))
        rear_grip = self.grasps[0].grip.copy()
        rear_grip[1, 3] += stations[0]
        result[ITEM] = inverse(rear_grip)
        for side, grasp, shift, station in zip(SIDES, self.grasps, reaches, stations):
            delta = (item @ grasp.grip)[:3, 3] + item[:3, 1] * station - (grasp.shoulder + shift)
            distance = np.linalg.norm(delta)
            constraint = 1. - freed if side == 'L' else 1.
            item[:3, 3] -= unit(delta) * max(0., distance - sum(grasp.segments) + 1.8) * constraint
        for side, grasp, shift, station in zip(SIDES, self.grasps, reaches, stations):
            offset = matrix(np.eye(3), shift)
            sliding_item = item.copy()
            sliding_item[:3, 3] += item[:3, 1] * station
            hint = (32., -15., -9.) if side == 'R' else (-15., -25., -16.)
            angles = np.array((-.22, 0., .22))
            if side == 'R':
                sliding_item = sliding_item @ bank
                angles += np.radians(-54. + 58.*push + 21.*freed)
            elbow = (self.camera @ np.r_[hint, 1.])[:3] - shift
            if side == 'L' and release > 0.:
                hand = sliding_item @ grasp.grip
                waiting = (self.camera @ np.array((-18., -30., -22., 1.)))[:3]
                hand[:3, 3] = lerp(hand[:3, 3], waiting, freed)
                palm = hand[:3, :3] @ grasp.palm_along
                relaxed = self.camera[:3, :3] @ unit(np.array((.36, .68, -.64)))
                turn = matrix(swing(palm, relaxed) @ hand[:3, :3], hand[:3, 3])
                sliding_item = blend(hand, turn, freed) @ inverse(grasp.grip)
            arm, _ = grasp.fit(inverse(offset) @ sliding_item, elbow, angles)
            if side == 'L' and release > 0.:
                arm = (*arm[:-1], relaxed_wrist(arm[-2], arm[-1], grasp, freed))
            arm = seat_wrist(arm, grasp, .84 if side == 'R' else .72*(1.-freed))
            parent = self.idle.world(self.rig.parents[grasp.names[0]])
            for name, transform in zip(grasp.names, arm):
                world = offset @ transform
                result[name] = inverse(parent) @ world
                parent = world
            result.update(grasp.fingers)
            if side == 'L' and release > 0.:
                # Fingers part before the palm clears the shaft and close after
                # it returns. The free-hand shape comes from the DM regrip.
                for name, closed in grasp.fingers.items():
                    result[name] = blend(closed, self.open_hand.local(name), ease(release * 2.4))
        return result

    def regrip(self, u, staff):
        # Transfer the released hand's real off-shaft excursion from B_out.
        # Its timing, opening fingers and reclosure survive the shorter reach.
        # The right hand stays mounted while its elbow brings the butt into view.
        u = float(np.clip(u, 0., 1.))
        if u >= 1.:
            return self.guard
        goal = LocalPose(self.rig, self.guard)
        source = Pose(self.rig, self.channels, self.release + (59. - 16. * ease((u - .12) / .88)) / 33.)
        if u <= 0.:
            return {n: staff.local(n) for n in self.names}
        start_item, end_item = staff.world(ITEM), goal.world(ITEM)
        roll = ease(u)
        item = blend(start_item, end_item, roll)
        right, left = self.grasps
        rear = blend(inverse(staff.local(ITEM)), inverse(goal.local(ITEM)), roll)
        right_path = lerp(staff.world(right.names[-1])[:3, 3], goal.world(right.names[-1])[:3, 3], roll)
        # Bring the supporting hand into view before the left fingers open.
        # The two ends of the path meet the actual staff swing's current pose.
        present = (self.camera @ np.array((15., -5., -30., 1.)))[:3]
        right_path = lerp(right_path, present, np.sin(np.pi*u)**2)
        item[:3, 3] = right_path - item[:3, :3] @ rear[:3, 3]
        rel = inverse(source.world(ITEM)) @ source.world(left.names[-1])
        clearance = np.clip((np.linalg.norm(rel[[0, 2], 3]) - 7.) / 35., 0., 1.)
        old_left = inverse(start_item) @ staff.world(left.names[-1])
        new_left = inverse(end_item) @ goal.world(left.names[-1])
        grasp = blend(old_left, new_left, ease((u - .18) / .68))
        free = item @ grasp
        free_path = lerp(staff.world(left.names[-1])[:3, 3], goal.world(left.names[-1])[:3, 3], roll)
        free_path += self.camera[:3, :3] @ np.array((-3., -3., 1.))
        free[:3, 3] = lerp(free[:3, 3], free_path, clearance)
        relaxed = self.camera[:3, :3] @ unit(np.array((.42, .58, -.70)))
        turned = matrix(swing(free[:3, :3] @ left.palm_along, relaxed) @ free[:3, :3], free[:3, 3])
        free = blend(free, turned, clearance * np.sin(np.pi*u))
        result = {name: blend(staff.local(name), goal.local(name), roll) for name in self.names}
        result[ITEM] = inverse(rear)
        body = LocalPose(self.rig, result)
        result.update(steer_arm(staff, goal, right, item @ rear, roll,
                                parent=body.world(self.rig.parents[right.names[0]])))
        result.update(steer_arm(staff, goal, left, free, roll, clearance * np.sin(np.pi*u),
                                parent=body.world(self.rig.parents[left.names[0]])))
        for name in left.fingers:
            result[name] = blend(result[name], source.local(name), clearance * np.sin(np.pi*u))
        return result

    def beat(self, t, path):
        for (start, a), (end, b) in zip(path, path[1:]):
            if t <= end:
                return self.strike(*lerp(np.array(a), np.array(b), ease((t - start) / (end - start))))
        return self.strike(*path[-1][1])

    def staff_edge(self, source, time, progress):
        """Cross from diagonal guard to the source using the DM free-hand path."""
        return self.regrip(1. - ease(progress), Pose(self.rig, source, time))


def movement_overlay(spear, path, target):
    """Keep the DM gait's phase and relative motion in the diagonal guard."""
    stream = load(path)
    source = tracks(path)
    times = source[ITEM][0][:, 0]
    def camera_item(t):
        pose = Pose(spear.rig, source, t)
        return inverse(pose.world('Camera')) @ pose.world(ITEM)
    rest = camera_item(times[0])
    arm_names = {ITEM, *(name for g in spear.grasps for name in (*g.names, *g.fingers))}
    poses = []
    for t in times:
        movement = camera_item(t)
        delta = movement[:3, 3] - rest[:3, 3]
        sway = movement[:3, 1] - rest[:3, 1]
        poses.append(spear.strike(-.45*delta[2], 0., .50*delta[0] + 2.*sway[0],
                                  .50*delta[1] + 2.*sway[1]))
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    for name, controller in zip(names, stream.root.controllers):
        if name not in arm_names:
            continue
        rotations = np.array([matrix_quat(p[name][:3, :3]) for p in poses])
        for i in range(1, len(rotations)):
            if rotations[i-1] @ rotations[i] < 0:
                rotations[i] *= -1
        controller.data.rotations.keys = np.column_stack((times, rotations)).astype(np.float32)
        controller.data.translations.keys = np.column_stack((times, [p[name][:3, 3] for p in poses])).astype(np.float32)
    stream.save(target / path.name)
    shutil.copyfile(path.with_suffix('.yaml'), target / path.with_suffix('.yaml').name)


def staff_back_hand(stream, channels, rig, vanilla, release):
    """Support the jab from underneath while the right hand rolls the staff.

    The shaft spins around its length during the lunge. Carrying the same roll
    into the supporting wrist would bring the elbow across the camera. Transport
    its contact frame along the shaft direction, and release on the original beat.
    """
    grasp = Grasp(vanilla, 'L', False)
    grasp.grip = matrix(turn_about((0, 1, 0), -165.), np.zeros(3)) @ grasp.grip
    grasp.grip[1, 3] = -36.
    grasp.pole = unit(np.array((-.8, -.58, .15)))
    grasp.wrist_bias = 6.
    reference = Pose(rig, channels, release + 39. / 33.).world(ITEM)
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    controls = dict(zip(names, stream.root.controllers))
    for i, t in enumerate(channels[grasp.names[-1]][0][:, 0]):
        frame = (float(t) - release) * 33.
        if not 20. < frame < 50.:
            continue
        w = ease(min((frame - 20.) / 8., (50. - frame) / 7.))
        pose = Pose(rig, channels, t)
        item = pose.world(ITEM).copy()
        item[:3, :3] = swing(reference[:3, 1], item[:3, 1]) @ reference[:3, :3]
        target = item @ grasp.grip
        shoulder = pose.world(grasp.names[1])[:3, 3]
        delta = target[:3, 3] - shoulder
        advance = max(0., np.linalg.norm(delta) - sum(grasp.segments) + 1.)
        offset = matrix(np.eye(3), shoulder - grasp.shoulder + unit(delta) * advance)
        chain, _ = grasp.fit(inverse(offset) @ item)
        local = {}
        parent = pose.world(rig.parents[grasp.names[0]])
        for joint, transform in zip(grasp.names, chain):
            world = offset @ transform
            local[joint] = inverse(parent) @ world
            parent = world
        goal = LocalPose(rig, {**{n: pose.local(n) for n in channels}, **local})
        hand = blend(pose.world(grasp.names[-1]), target, w)
        arm = steer_arm(pose, goal, grasp, hand, w)
        arm.update({n: blend(pose.local(n), value, w) for n, value in grasp.fingers.items()})
        for joint, value in arm.items():
            data = controls[joint].data
            q = matrix_quat(value[:3, :3])
            if data.rotations.keys[i-1, 1:] @ q < 0.:
                q = -q
            data.rotations.keys[i, 1:] = q
            data.translations.keys[i, 1:] = value[:3, 3]


def build(arms, native_skeleton, native_animation):
    folder = arms / 'Animations/brutal_fp_staff'
    source = folder / 'weapononehand.kf'
    target = folder.with_name('brutal_fp_spear')
    target.mkdir(exist_ok=True)
    stream = load(source)
    text = keys(stream)
    marks = {line.removeprefix('weapononehand: '): float(t)
             for t, label in text.keys for line in label.splitlines()}
    native_rig, native_tracks = Rig(native_skeleton), tracks(native_animation)
    starts = native_starts(native_animation)
    vanilla = Pose(native_rig, native_tracks, starts['idle2w: start'])
    fist = Pose(native_rig, native_tracks, starts['idlehh: start'])
    channels = tracks(source)
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    profile = json.loads((arms / 'staff-native-grasp-profile.json').read_text())
    staff_back_hand(stream, channels, rig, vanilla, marks['thrust min hit'])
    stream.save(source)
    spear = Spear(rig, channels, marks, vanilla, fist, profile)
    guard = (0., 0., 0., 0., 0.)
    charged = [(0., guard), (.085, (-5.5, 0., 4., -3.8, .16)),
               (.225, (26., 1., -.5, 1., 1.)), (.300, (26., 1., -.5, 1., 1.)),
               (.50, guard)]
    segments = []
    labels = []

    def section(name, start, length, motion, events):
        segments.append((start, start + length, motion))
        labels.extend((start + t, 'weapononehand: ' + name + ' ' + event) for t, event in events)

    start, maximum, release = (marks['thrust ' + key] for key in ('start', 'max attack', 'min hit'))
    drawn = (-6., .25, 4., -3.5, .0)
    preparation = [(0., guard), (.38, drawn), (maximum-start, drawn)]
    charged[0] = (0., drawn)
    section('thrust', start, maximum - start, lambda t: spear.beat(t, preparation),
            [(0., 'start'), (marks['thrust min attack'] - start, 'min attack'), (maximum - start, 'max attack')])
    end = .55
    section('thrust', release, end, lambda t: spear.beat(t, charged),
            [(0., 'min hit'), (.211, 'hit'), (.46, 'chain')] +
            [(t, length + ' follow ' + edge) for t, edge in ((.212, 'start'), (end, 'stop'))
             for length in ('small', 'medium', 'large')])
    hold = marks['thrust hold start']
    held_pose = spear.strike(*drawn)
    section('thrust hold', hold, .22, lambda t: held_pose,
            [(0., 'start'), (.02, 'loop start'), (.22, 'loop stop'), (.22, 'stop')])
    labels.extend([(hold + .02, 'weapononehand: loop start'), (hold + .22, 'weapononehand: loop stop')])
    # Four strokes share the standing guard: centre, inside-low, high outside
    # and a fully released finishing thrust.
    attacks = [
        ('a', .42, .20, (20., .84, -4., -5., .70), (-2.5, 0., -2., -2., 0.)),
        ('b', .39, .205, (17., .76, 8., -1., .42), (-4., .05, 9., -7., 0.)),
        ('c', .41, .265, (16., .72, 3.5, 7.5, .55), (-3., 0., 5., 8.5, .12)),
        ('d', .49, .295, (25., 1., -5., 3., 1.), (-7., 0., 9., -6., .12)),
    ]
    for letter, duration, hit, extended, drawn in attacks:
        draw = .070 if letter == 'a' else .105
        path = [(0., guard), (draw, drawn), (hit + .016, extended), (hit + .038, extended), (duration, guard)]
        if letter == 'd':
            path.insert(2, (.18, (-5., .06, 6.5, -3., .34)))
            path[-2] = (hit + .075, extended)

        def fast(t, path=path):
            return spear.beat(t, path)

        name = 'fast' + letter
        section(name, marks[name + ' start'], duration, fast,
                [(0., 'start'), (draw, 'swing'), (hit, 'thrust hit'), (duration, 'stop')])
        endname = 'fastend' + letter
        endstart = marks.get(endname + ' start', marks['bouncer stop'] + .1)
        section(endname, endstart, .22, lambda t: spear.guard,
                [(0., 'start'), (.22, 'stop')])

    # A tap briefly starts the charged windup before the engine recognises a
    # fast attack. Keep that grace period in guard; a held press completes the
    # regrip, then meets the authored staff tracks before their action keys.
    def join(time, length, entering, grace=0.):
        def motion(t):
            progress = max(0., (t-grace)/(length-grace)) if entering else 1.-t/length
            return spear.staff_edge(channels, time+t, progress)
        segments.append((time, time+length, motion))
    def source_pose(time):
        pose = Pose(rig, channels, time)
        return {n: pose.local(n) for n in channels}

    def staff_attack(name):
        start = marks[name + ' start']
        maximum = marks[name + ' max attack']
        # Reach the resting transverse hold before playing the source windup.
        # Fade the reserved transition time out by maximum charge. The end
        # of the original windup and the hit clocks thus meet exactly.
        change = .46
        def enter(t):
            if t < change:
                return spear.regrip(1. - ease((t-.085)/(change-.085)), spear.idle)
            u = (t-change)/(maximum-start-change)
            return source_pose(start+t-change*(1.-ease(u)))
        segments.append((start, maximum, enter))
        last_hit = marks.get(name+' hit2', marks[name+' hit'])
        stop = marks[name+' large follow stop']
        tail = stop-last_hit
        def leave(t):
            if t < tail-change:
                return source_pose(last_hit+t+change*ease(t/(tail-change)))
            return spear.regrip(ease((t-tail+change)/change), spear.idle)
        segments.append((last_hit, stop, leave))
    for name in ('slash', 'chop', 'neutral'):
        staff_attack(name)
    join(marks['equip stop']-.32, .32, False)
    join(marks['unequip start'], .24, True)

    # Different source sections share the KF but are played independently.
    # Preserve original key grids outside each replaced interval.
    replaced = ('thrust ', 'fasta ', 'fastb ', 'fastc ', 'fastd ', 'fastend')
    for t, label in text.keys:
        kept = [line for line in label.splitlines() if not line.removeprefix('weapononehand: ').startswith(replaced)
                and not (hold <= t <= marks['thrust hold stop'] and line in ('weapononehand: loop start', 'weapononehand: loop stop'))]
        if kept:
            labels.append((float(t), '\r\n'.join(kept)))
    labels.append((0., 'weapononehand: brutal spear grip'))
    labels.sort(key=lambda pair: pair[0])
    text.keys = np.array(labels, dtype=text.keys.dtype)
    samples = {}
    for begin, stop, fn in segments:
        times = np.linspace(begin, stop, max(2, int(np.ceil((stop - begin) * 60)) + 1))
        for t in times:
            samples[float(t)] = fn(float(t - begin))
    times = np.array(sorted(samples))
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    for name, controller in zip(names, stream.root.controllers):
        for field, values in (
                (controller.data.rotations, [matrix_quat(samples[t][name][:3, :3]) for t in times]),
                (controller.data.translations, [samples[t][name][:3, 3] for t in times])):
            old = field.keys
            keep = np.ones(len(old), dtype=bool)
            for begin, stop, _ in segments:
                keep &= (old[:, 0] < begin - 1e-5) | (old[:, 0] > stop + 1e-5)
            merged = np.vstack((old[keep], np.column_stack((times, values)))).astype(np.float32)
            merged = merged[np.argsort(merged[:, 0])]
            if field is controller.data.rotations:
                for i in range(1, len(merged)):
                    if merged[i - 1, 1:] @ merged[i, 1:] < 0.:
                        merged[i, 1:] *= -1
            field.keys = merged
        controller.stop_time = max(controller.stop_time, times[-1])
    stream.save(target / source.name)
    shutil.copyfile(source.with_suffix('.yaml'), target / 'weapononehand.yaml')
    for path in sorted(folder.glob('*1h.kf')):
        if path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            movement_overlay(spear, path, target)
    manifest_path = arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())
    for path in (source, *sorted(target.iterdir())):
        manifest['outputs'][path.relative_to(arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['spear'] = {'charged': 'thrust', 'quick': [row[0] for row in attacks],
                         'base': 'brutal_fp_staff', 'native_grip': 'idle2w'}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('arms', 'native-skeleton', 'native-animation'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    build(args.arms, args.native_skeleton, args.native_animation)
