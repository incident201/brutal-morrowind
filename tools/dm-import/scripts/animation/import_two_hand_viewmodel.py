#!/usr/bin/env python3
"""Retarget the imported first-person sword motion around Morrowind's paired grips."""
import argparse
import json
from pathlib import Path
import re

import numpy as np

from audit_staff_mesh_grip import Rig, matrix, nif, slerp
from audit_viewmodel_continuity import tracks
from retarget_kf import cross, matrix_quat, norm, quat_matrix


ITEM = 'Weapon Bone'
ROOT = 'ModelRoot'
SIDES = ('R', 'L')
ARM = ('Clavicle', 'UpperArm', 'Forearm', 'Hand')
SWIVELS = np.linspace(-1.2, 1.2, 17)
EYE = np.array((0.02, 5.88, 124.71))
# Offset from the paired grip, outward tilt, and time to finish easing out after contact.
WINDUPS = {
    'neutral': ((2., 9., -23.), 0., .15),
    'slashright': ((16., 7., -7.), 24., 0.),
    'slash': ((2., 6., -5.), -6., .10),
}
EQUIP_TIMING = (0.11, 0.44, 0.68)  # attach, left-hand catch, stop
SWORD_DRAW_TIMING = (.26, .72, .94)  # grasp behind the shoulder, left-hand catch, stop
# Target KF durations for the authored Dark Messiah fast combo (played at 0.86x for attacks,
# 1.0x for fastenda). Preserves all 5 authored swings (A..E), fastenda and transA..E continuity.
FAST_TARGETS = {
    'fasta': 0.46,
    'fastb': 0.50,
    'fastc': 0.50,
    'fastd': 0.50,
    'faste': 0.48,
    'fastenda': 0.56,
}


def bone(side, joint):
    return f'Bip01 {side} {joint}'


def inverse(transform):
    rotation = transform[:3, :3].T
    return matrix(rotation, -rotation @ transform[:3, 3])


def unit(vector):
    return vector / norm(vector)


def basis(axis, normal):
    axis = unit(axis)
    normal = unit(normal - axis * (axis @ normal))
    return np.column_stack((axis, cross(normal, axis), normal))


def rotate_vec(src, dst):
    """Shortest-arc rotation matrix taking unit vector src to unit vector dst."""
    src, dst = unit(src), unit(dst)
    c = float(np.clip(src @ dst, -1., 1.))
    if c > 1. - 1e-7:
        return np.eye(3)
    axis = cross(src, dst)
    s = norm(axis)
    if s < 1e-7:
        return np.eye(3)
    axis = axis / s
    angle = np.arctan2(s, c)
    return turn_about(axis, np.degrees(angle))


def scale_turn(quaternion, weight):
    # Keep the supplied hemisphere so a swing which crosses 180 degrees
    # continues in the same direction. Shortest-path slerp would flip it.
    vector = quaternion[1:]
    length = norm(vector)
    if length < 1e-7:
        return np.eye(3)
    half_angle = weight * np.arctan2(length, quaternion[0])
    return quat_matrix(np.r_[np.sin(half_angle) * vector / length, np.cos(half_angle)])


def turn_about(axis, degrees):
    half = np.radians(degrees) / 2
    return quat_matrix(np.r_[np.asarray(axis) * np.sin(half), np.cos(half)])


def sample(keys, time, count, interpolate):
    index = int(np.searchsorted(keys[:, 0], time, side='right'))
    if not index or index == len(keys):
        return keys[max(0, index - 1), 1:count + 1]
    first, second = keys[index - 1:index + 1]
    fraction = (time - first[0]) / (second[0] - first[0])
    return interpolate(first[1:count + 1], second[1:count + 1], fraction)


def lerp(a, b, weight):
    return a + weight * (b - a)


def smoothstep(weight):
    weight = np.clip(weight, 0., 1.)
    return weight * weight * (3. - 2. * weight)


def blend_transform(first, second, weight):
    a, b = (matrix_quat(pose[:3, :3]) for pose in (first, second))
    return matrix(quat_matrix(slerp(a, b, weight)[[1, 2, 3, 0]]), lerp(first[:3, 3], second[:3, 3], weight))


def charge_weight(time, marks, draw_time, return_start, return_stop):
    if marks['hold start'] <= time <= marks['hold stop']:
        return 1.
    start = marks['start']
    if not start < time < return_stop:
        return 0.
    return smoothstep((time - start) / draw_time) * smoothstep((return_stop - time) / (return_stop - return_start))


def blend_with_sword(pose, paired, weight, *, sword_grip, left_pose=None):
    # The thrust needs the sword's own wrist compensation to follow the line
    # of aim. The chop can keep the closed paired fist and weapon socket.
    for name, transform in paired.items():
        if not sword_grip and (name == ITEM or name.startswith(bone('R', 'Finger'))):
            continue
        source = left_pose if left_pose is not None and name.startswith(bone('L', '')) else pose
        sword = source.local(name)
        if weight >= 1.:
            paired[name] = sword
        else:
            paired[name] = blend_transform(transform, sword, weight)


class Pose:
    def __init__(self, rig, channels, time):
        self.rig = rig
        self.channels = channels
        self.time = time
        self.cache = {ROOT: np.eye(4)}

    def local(self, name):
        node = self.rig.nodes[name]
        rotation, position = node.rotation, node.translation
        if name in self.channels:
            rotations, translations = self.channels[name]
            if len(rotations):
                rotation = quat_matrix(sample(rotations, self.time, 4, slerp)[[1, 2, 3, 0]])
            if len(translations):
                position = sample(translations, self.time, 3, lerp)
        return matrix(rotation, position)

    def world(self, name):
        if name not in self.cache:
            self.cache[name] = self.world(self.rig.parents[name]) @ self.local(name)
        return self.cache[name]


def blend_thrust(pose, paired, section, weight):
    # The last part of the sword's draw pulls a short axe head out of view.
    # Stop the draw before that point and keep the extension on release.
    start, peak, release = section['start'], section['max attack'], section['min hit']
    draw_brake = .30
    brake_start = peak - draw_brake
    extension_start, extension_end = release + .05, release + .16
    time = pose.time
    sword, left = pose, None
    if time <= peak:
        if time > brake_start:
            drawn = time - (time - brake_start) ** 2 / (2 * draw_brake)
            sword = Pose(pose.rig, pose.channels, drawn)
        # Stop the free hand while it is still on the left. The sword donor's
        # later reach would cross the less deeply drawn right wrist.
        left_draw = section['min attack'] - start
        remaining = (peak - time) / (peak - start)
        left_time = start + left_draw * (1. - remaining ** ((peak - start) / left_draw))
        left = Pose(pose.rig, pose.channels, left_time)
    elif time < extension_end or section['hold start'] <= time <= section['hold stop']:
        sword = Pose(pose.rig, pose.channels, peak - draw_brake / 2)
        left = Pose(pose.rig, pose.channels, section['min attack'])
    blend_with_sword(sword, paired, weight, sword_grip=True, left_pose=left)
    if extension_start < time < extension_end:
        extension = smoothstep((time - extension_start) / (extension_end - extension_start))
        blend_with_sword(pose, paired, extension, sword_grip=True)


class Grasp:
    def __init__(self, native, side, blunt):
        self.names = [bone(side, joint) for joint in ARM]
        self.clavicle, upper, fore, hand = map(native.world, self.names)
        self.clav_root = self.clavicle[:3, 3].copy()
        self.shoulder, self.elbow, self.wrist = upper[:3, 3], fore[:3, 3], hand[:3, 3]
        self.clav_vec = self.shoulder - self.clav_root
        self.clav_len = norm(self.clav_vec)
        self.grip = inverse(native.world(ITEM)) @ hand
        self.segments = (norm(self.elbow - self.shoulder), norm(self.wrist - self.elbow))
        axis = unit(self.wrist - self.shoulder)
        self.wrist_axis = axis
        self.pole = unit(self.elbow - self.shoulder - axis * (axis @ (self.elbow - self.shoulder)))
        normal = cross(self.elbow - self.shoulder, self.wrist - self.elbow)
        self.upper_frame = basis(self.elbow - self.shoulder, normal).T @ upper[:3, :3]
        self.fore_frame = basis(self.wrist - self.elbow, normal).T @ fore[:3, :3]
        self.wrist_frame = fore[:3, :3].T @ hand[:3, :3]
        self.fingers = {name: native.local(name) for name in native.rig.nodes
                        if name.startswith(bone(side, 'Finger'))}
        row = [self.fingers[bone(side, f'Finger{digit}')][:3, 3] for digit in range(1, 5)]
        self.palm_along = unit(np.mean(row, axis=0))
        self.wrist_bias = .85 if blunt else .25
        if blunt and side == 'L':
            # Axe and hammer hafts have room below the vanilla close grip.
            # Keep the same closed fingers and radial contact on the shaft.
            self.grip[1, 3] -= 7.

    def shoulder_pose(self, shift=None):
        if shift is None or norm(shift) < 1e-6:
            return self.clavicle, self.shoulder
        new_dir = unit(self.clav_vec + shift)
        rot = rotate_vec(self.clav_vec, new_dir)
        return matrix(rot @ self.clavicle[:3, :3], self.clav_root), self.clav_root + new_dir * self.clav_len

    def fit_hand(self, hand, elbow_hint=None, swivels=SWIVELS, smooth_bend=False, shoulder_shift=None):
        clavicle, shoulder = self.shoulder_pose(shoulder_shift)
        wrist = hand[:3, 3]
        delta = wrist - shoulder
        distance = norm(delta)
        axis = delta / distance
        upper_length, fore_length = self.segments
        projection = (distance ** 2 + upper_length ** 2 - fore_length ** 2) / (2 * distance)
        height = np.sqrt(max(0., upper_length ** 2 - projection ** 2))
        base = shoulder + axis * projection
        guide = self.pole if elbow_hint is None else elbow_hint - shoulder
        if smooth_bend:
            # Carry the bind elbow plane with the wrist axis. Projecting a
            # fixed pole can flip the elbow when a wide swing aligns with it.
            alignment = 1. + self.wrist_axis @ axis
            if alignment > 1e-6:
                guide = self.pole - (self.pole @ axis) / alignment * (self.wrist_axis + axis)
        pole = unit(guide - axis * (axis @ guide))
        tangent = cross(axis, pole)
        desired_fore = hand[:3, :3] @ self.wrist_frame.T
        palm_along = hand[:3, :3] @ self.palm_along

        def frames(angle):
            bend = np.cos(angle) * pole + np.sin(angle) * tangent
            elbow = base + height * bend
            fore_axis = unit(wrist - elbow)
            # Let axial forearm rotation absorb palm roll while respecting the
            # bent elbow. The angle between forearm and native palm stays measurable.
            flex = np.arccos(np.clip(fore_axis @ desired_fore[:, 0], -1., 1.))
            palm_flex = np.arccos(np.clip(fore_axis @ palm_along, -1., 1.))
            # The smooth swing solver carries the elbow plane through full
            # extension, where the arm segments can become collinear.
            normal = cross(bend, axis) if smooth_bend else cross(elbow - shoulder, wrist - elbow)
            upper_rotation = basis(elbow - shoulder, normal) @ self.upper_frame
            fore_rotation = basis(wrist - elbow, normal) @ self.fore_frame
            twist_sine = fore_rotation[:, 2] @ desired_fore[:, 1] - fore_rotation[:, 1] @ desired_fore[:, 2]
            if smooth_bend:
                # Let the twist recede as the forearm and wrist axes oppose
                # each other, where the angle of the projected wrist is unstable.
                twist = .525 * twist_sine
            else:
                twist = np.arctan2(twist_sine, fore_rotation[:, 1] @ desired_fore[:, 1]
                                   + fore_rotation[:, 2] @ desired_fore[:, 2])
            return flex, twist, palm_flex, upper_rotation, fore_rotation, elbow

        if smooth_bend:
            # Nudge the elbow away from the desired forearm direction so the
            # wrist can follow the grip. Keep enough of the transported pole
            # to guide the bend through a reversal of the wrist.
            fore_hint = (desired_fore[:, 0] + self.wrist_bias * palm_along) / (1. + self.wrist_bias)
            angle = np.arctan2(-.55 * (tangent @ fore_hint), 1. - .55 * (pole @ fore_hint))
        else:
            flex, twist, palm_flex = np.array([frames(angle)[:3] for angle in swivels]).T
            costs = flex ** 2 + .23 * twist ** 2 + .38 * swivels ** 2 + self.wrist_bias * palm_flex ** 2
            weights = np.exp(-12 * (costs - costs.min()))
            angle = weights @ swivels / weights.sum()
        flex, twist, palm_flex, upper_rotation, fore_rotation, elbow = frames(angle)
        twist = np.clip(twist, -1.05, 1.05)
        c, s = np.cos(twist), np.sin(twist)
        fore_rotation = fore_rotation @ np.array(((1., 0., 0.), (0., c, -s), (0., s, c)))
        return (clavicle, matrix(upper_rotation, shoulder), matrix(fore_rotation, elbow), hand), max(
            flex, palm_flex - .18)

    def fit(self, item, elbow_hint=None, swivels=SWIVELS, smooth_bend=False, shoulder_shift=None):
        return self.fit_hand(item @ self.grip, elbow_hint=elbow_hint, swivels=swivels,
                             smooth_bend=smooth_bend, shoulder_shift=shoulder_shift)


def keys(stream):
    return next(extra for extra in stream.root.extra_datas if isinstance(extra, nif.NiTextKeyExtraData))


def load(path):
    stream = nif.NiStream()
    stream.load(path)
    return stream


def native_starts(path):
    return {key.lower(): float(time) for time, text in keys(load(path)).keys for key in text.splitlines() if key}


def eval_key_curve(u, knots):
    """Evaluate a piecewise smoothstep-interpolated (t, pos, euler_xyz) keyframe curve."""
    if u <= knots[0][0]:
        return np.asarray(knots[0][1], float), tuple(knots[0][2])
    if u >= knots[-1][0]:
        return np.asarray(knots[-1][1], float), tuple(knots[-1][2])
    for i in range(len(knots) - 1):
        t0, p0, r0 = knots[i]
        t1, p1, r1 = knots[i + 1]
        if t0 <= u <= t1:
            f = smoothstep((u - t0) / max(1e-6, t1 - t0))
            pos = lerp(np.asarray(p0, float), np.asarray(p1, float), f)
            rot = tuple(lerp(a, b, f) for a, b in zip(r0, r1))
            return pos, rot
    return np.asarray(knots[-1][1], float), tuple(knots[-1][2])


def rot_xyz(rx, ry, rz):
    """World-space pitch (X), roll (Y), yaw (Z) rotation in degrees."""
    return turn_about((0, 0, 1), rz) @ turn_about((0, 1, 0), ry) @ turn_about((1, 0, 0), rx)


def soft_min(value, ceiling, softness=4.0):
    """Smooth C-infinity upper bound that approaches value below ceiling without hard clamping."""
    diff = (ceiling - value) / softness
    if diff > 20.:
        return value
    if diff < -20.:
        return ceiling
    return ceiling - softness * np.log1p(np.exp(diff))


def soft_max(value, floor, softness=3.5):
    """Smooth C-infinity lower bound without hard clamping."""
    return -soft_min(-value, -floor, softness)


def warp_weight_u(u_dst, strength=0.36):
    """Monotonic weight curve u_dst -> u_src: slower anticipation & follow-through, brisker mid-strike."""
    u = float(np.clip(u_dst, 0., 1.))
    return u - strength * np.sin(2. * np.pi * u) / (2. * np.pi)


def inv_warp_weight_u(u_src, strength=0.36):
    """Inverse of warp_weight_u: maps a source fraction u_src to its warped destination fraction."""
    grid = np.linspace(0., 1., 513)
    src = grid - strength * np.sin(2. * np.pi * grid) / (2. * np.pi)
    return float(np.interp(np.clip(u_src, 0., 1.), src, grid))


# Clean over-the-back draw: pull over right shoulder, left hand catches at t=0.44, subtle settle to idle2c.
EQUIP_KNOTS = (
    (0.00, ( 13.5, -18.5,  15.5), ( 56., -24., -16.)),
    (0.11, ( 13.0, -17.0,  16.5), ( 50., -22., -14.)),
    (0.27, ( 10.5,  -7.5,  10.5), ( 20., -12.,  -6.)),
    (0.44, (  1.5,  -0.5,  -2.0), ( -4.,   2.,   0.)),
    (0.56, (  0.3,   0.1,  -0.5), ( -1.,   0.,   0.)),
    (0.68, (  0.0,   0.0,   0.0), (  0.,   0.,   0.)),
)


class Retarget:
    def __init__(self, rig, sword_idle, native, blunt, empty):
        self.rig = rig
        self.source_idle = sword_idle.world(ITEM)
        self.idle = native.world(ITEM).copy()
        self.idle[:3, 3] += (-4., 9., 4.)
        self.idle[:3, :3] = turn_about((1, 0, 0), -18) @ self.idle[:3, :3]
        self.stance_offset = matrix_quat(self.source_idle[:3, :3].T @ self.idle[:3, :3])
        self.grasps = [Grasp(native, side, blunt) for side in SIDES]
        self.item_mount = inverse(self.grasps[0].grip)
        self.empty = empty

    def sword_transfer_pose(self, pose, time):
        """One continuous reach, pull and catch, sampled in reverse to stow."""
        attach, catch, stop = SWORD_DRAW_TIMING
        guard = self.idle
        stowed = matrix(turn_about((1., 0., 0.), 164.) @ guard[:3, :3], (21., -15., 132.))
        supported = guard.copy()
        supported[:3, 3] += (4., -3., 3.)
        supported, _ = self.fit_item(supported)
        right, left = self.grasps
        empty_right, empty_left = (self.empty.world(bone(side, 'Hand')) for side in SIDES)

        if time <= attach:
            # Bring the empty hand up the right edge before reaching back.
            reach = smoothstep(time / attach)
            grip = stowed @ right.grip
            hand = blend_transform(empty_right, grip, reach)
            high = np.array((27., 30., 127.))
            hand[:3, 3] = lerp(lerp(empty_right[:3, 3], high, reach),
                                lerp(high, grip[:3, 3], reach), reach)
            arms = [right.fit_hand(hand), left.fit_hand(empty_left)]
        elif time < catch:
            pull = smoothstep((time - attach) / (catch - attach))
            # Draw along the back first. Turn the long blade out around the
            # right shoulder as the hilt comes forward.
            high, forward = np.array(((28., -7., 147.), (33., 23., 135.)))
            start = lerp(stowed[:3, 3], high, pull)
            middle = lerp(high, forward, pull)
            end = lerp(forward, supported[:3, 3], pull)
            position = lerp(lerp(start, middle, pull), lerp(middle, end, pull), pull)
            rotation = (turn_about((0., 0., 1.), 38. * np.sin(np.pi * pull))
                        @ turn_about((1., 0., 0.), 164. * (1. - pull)) @ guard[:3, :3])
            item, arms = self.fit_item(matrix(rotation, position), one_hand=True)
            meet = smoothstep((time - .48) / (catch - .48))
            free = blend_transform(empty_left, item @ left.grip, meet)
            arms.append(left.fit_hand(free))
        else:
            settle = smoothstep((time - catch) / (stop - catch))
            _, arms = self.fit_item(blend_transform(supported, guard, settle))
        result = self.pack_arms(pose, arms)
        # Start and finish on the lowered idle's actual arms and fingers.
        # Keep the weapon socket fixed in the palm through the hidden reach.
        if time < .12:
            mount = result[ITEM]
            blend_with_sword(self.empty, result, 1. - smoothstep(time / .12), sword_grip=True)
            result[ITEM] = mount
        return result

    def fit(self, turn, position, weight, contest):
        # Keep the same, rigid item-space offsets for both palms. Bring the
        # entire pair into reach before solving either arm.
        item = matrix(scale_turn(turn, weight) @ self.idle[:3, :3],
                      lerp(self.idle[:3, 3], position, weight))
        # Let the blade meet the opponent nearer the centre line. A long
        # weapon in the unaltered sword pose needs too much lateral reach.
        if contest:
            turn = turn_about((0, 1, 0), -34 * contest) @ turn_about((1, 0, 0), 9 * contest)
            item[:3, :3] = turn @ item[:3, :3]
            item[:3, 3] += np.array((-3., -2., -4.)) * contest
        return self.fit_item(item)

    def shoulder_shifts(self, item, weight):
        if weight <= 1e-6:
            return [None, None]
        shifts = []
        for grasp in self.grasps:
            wrist = (item @ grasp.grip)[:3, 3]
            delta = wrist - grasp.shoulder
            shift = np.array((
                np.clip(delta[0] * 0.14, -2.5, 2.5),
                np.clip((delta[1] - 16.0) * 0.45, 0.0, 6.5),
                -3.0,
            )) * weight
            shifts.append(shift)
        return shifts

    def fit_item(self, item, swing=False, one_hand=False, shoulder_weight=0.):
        grasps = self.grasps[:1] if one_hand else self.grasps
        shifts = self.shoulder_shifts(item, shoulder_weight if swing and not one_hand else 0.)
        shoulders = [grasp.shoulder_pose(shift)[1] for grasp, shift in zip(grasps, shifts)]
        for _ in range(4):
            for grasp, shoulder in zip(grasps, shoulders):
                wrist = (item @ grasp.grip)[:3, 3]
                delta = wrist - shoulder
                length = norm(delta)
                reachable = np.clip(length, 11. if swing else 8., sum(grasp.segments) - 1.3)
                item[:3, 3] += delta * (reachable / length - 1.)
        arms = [grasp.fit(item, smooth_bend=swing, shoulder_shift=shift)
                for grasp, shift in zip(grasps, shifts)]
        return item, arms

    def pack_arms(self, pose, arms):
        result = {ITEM: self.item_mount}
        for grasp, (chain, _) in zip(self.grasps, arms):
            parent = pose.world(self.rig.parents[grasp.names[0]])
            for name, transform in zip(grasp.names, chain):
                result[name] = inverse(parent) @ transform
                parent = transform
            result.update(grasp.fingers)
        return result

    def equip_pose(self, pose, local_t):
        """Clean over-the-back two-handed draw: right hand pulls over right shoulder,
        left hand rises from lower-left to catch the grip at EQUIP_TIMING[1]."""
        if self.empty is not None:
            return self.sword_transfer_pose(pose, local_t)
        _, catch_t, stop_t = EQUIP_TIMING
        t = float(np.clip(local_t, 0., stop_t))
        pos_off, (rx, ry, rz) = eval_key_curve(t, EQUIP_KNOTS)
        item = matrix(rot_xyz(rx, ry, rz) @ self.idle[:3, :3], self.idle[:3, 3] + pos_off)
        if t >= catch_t:
            _, arms = self.fit_item(item, swing=False)
            return self.pack_arms(pose, arms)
        # Right hand draws the weapon over the right shoulder with slight shoulder lift.
        r_shift = np.array((1.0, 1.5, 2.0)) * (1. - smoothstep(t / catch_t))
        r_grasp, l_grasp = self.grasps
        _, r_sh = r_grasp.shoulder_pose(r_shift)
        for _ in range(3):
            wrist = (item @ r_grasp.grip)[:3, 3]
            delta = wrist - r_sh
            length = norm(delta)
            reachable = np.clip(length, 9., sum(r_grasp.segments) - 1.3)
            item[:3, 3] += delta * (reachable / length - 1.)
        # Keep smooth_bend=False throughout equip so there is zero elbow/wrist pop at catch_t.
        r_arm = r_grasp.fit(item, smooth_bend=False, shoulder_shift=r_shift)
        # Left hand waits low-left and rises smoothly to meet the lower grip at catch_t.
        catch_w = smoothstep((t - 0.14) / max(1e-6, catch_t - 0.14))
        target_l = item @ l_grasp.grip
        idle_l = self.idle @ l_grasp.grip
        low_l_pos = idle_l[:3, 3] + np.array((-12.0, -8.0, -10.0))
        low_l_rot = turn_about((0, 0, 1), -18.) @ turn_about((1, 0, 0), -12.) @ idle_l[:3, :3]
        q0, q1 = matrix_quat(low_l_rot), matrix_quat(target_l[:3, :3])
        if q0 @ q1 < 0:
            q1 = -q1
        l_hand = matrix(quat_matrix(slerp(q0, q1, catch_w)[[1, 2, 3, 0]]),
                        lerp(low_l_pos, target_l[:3, 3], catch_w))
        l_arm = l_grasp.fit_hand(l_hand, smooth_bend=False)
        return self.pack_arms(pose, [r_arm, l_arm])

    def swing_item(self, source, delta, turn, falling, windup, slashright_arc=None, fast_swing=False):
        # The paired guard is higher than the sword's. As the hands leave it,
        # remove most of that offset so the authored swing crosses the view.
        angle = 2. * np.arccos(np.clip(abs(turn[0]), 0., 1.))
        motion = min(1., np.hypot(norm(delta) / 34., angle / 1.1))
        weight = motion * motion * (3. - 2. * motion)
        offset = slerp(self.stance_offset, np.array((1., 0., 0., 0.)), .72 * weight)
        rotation = source[:3, :3] @ quat_matrix(offset[[1, 2, 3, 0]])
        scale = (1.22, .96, .95) if fast_swing else (1., .88, .95)
        position = self.idle[:3, 3] + delta * scale + weight * np.array((0., 0., -22.))
        position[1] = max(13., position[1])
        if fast_swing:
            # Widen the authored Dark Messiah fast combo cleanly without hard wall clamps:
            # keep high fastC/fastD arcs smoothly below eye level and forward of the lens.
            position[1] = soft_max(position[1] + 3.5 * weight, EYE[1] + 17.5, softness=3.5)
            position[2] = soft_min(position[2], EYE[2] - 8.5, softness=4.0)
        if falling and rotation[1, 1] < 0.:
            # Compress the part of the arc behind the eye towards overhead.
            # The rate matches the unaltered forward arc at the vertical pose.
            down_from_vertical = np.pi / 2. - np.arctan2(rotation[2, 1], -rotation[1, 1])
            shortened = .20 * down_from_vertical - .256 * np.expm1(-down_from_vertical / .32)
            loft = np.degrees(down_from_vertical - shortened)
            raised = min(1., -rotation[1, 1] / .58)
            rotation = turn_about((1., 0., 0.), -loft) @ rotation
            # Leave room for the near plane and a broad axe head in the windup.
            position += raised * raised * (3. - 2. * raised) * np.array((16., 6., -3.))
        if windup is not None:
            offset, tilt, amount = windup
            rotation = turn_about((0, 1, 0), tilt * amount) @ rotation
            position += amount * np.asarray(offset)
        if slashright_arc is not None:
            # Smoothly lower the left-step charged slash plane across impact AND carry
            # its follow-through down-left off the bottom-left edge of the viewport
            # (instead of freezing on-screen or sweeping across the eyes).
            hit_w, follow_w = slashright_arc
            position[1] += 5.5 * hit_w
            position[2] -= 10.5 * hit_w
            if follow_w > 0.:
                position[0] -= 7.5 * follow_w
                position[2] -= 17.5 * follow_w
                rotation = turn_about((0, 1, 0), -28.0 * follow_w) @ rotation
        return matrix(rotation, position), weight

    def sprint_item(self, source, sword_start):
        # Carry the long weapon lower on the right. Follow the sword's cycle
        # with a short stroke so the tip stays clear of the centre of the view.
        travel = .20
        turn = matrix_quat(source[:3, :3] @ sword_start[:3, :3].T)
        carry = turn_about((0, 0, 1), -16) @ turn_about((1, 0, 0), 20) @ sword_start[:3, :3]
        position = np.array((23., 24., 95.)) + travel * (source[:3, 3] - sword_start[:3, 3])
        return matrix(scale_turn(turn, travel) @ carry, position)

    def adapt(self, pose, turn, attacking, contest, swing, falling, *,
              windup, running, sprint_start, slashright_arc=None, fast_swing=False):
        source = pose.world(ITEM)
        delta = source[:3, 3] - self.source_idle[:3, 3]
        # Transfer the sword's world-space swing about our different idle
        # orientation. Translation is measured from the same sword idle.
        if sprint_start is not None:
            # Keep the authored free left arm and fingers from the sword sprint.
            _, arms = self.fit_item(self.sprint_item(source, sprint_start), swing=True, one_hand=True)
        elif swing:
            item, motion = self.swing_item(
                source, delta, turn, falling, windup,
                slashright_arc=slashright_arc, fast_swing=fast_swing)
            # Keep neutral's low strike clearance (shoulders stay at bind height) while
            # protracting shoulders on side slashes and wide fast swings.
            sh_w = motion if (slashright_arc is not None or fast_swing or windup is None or windup[0][2] > -15.) else 0.
            _, arms = self.fit_item(item, swing=True, shoulder_weight=sh_w)
        else:
            travel = (.66, .53, .66) if attacking else (.47, .52, .53)
            position = self.idle[:3, 3] + delta * travel
            position[1] = max(17., position[1])
            weight = .32 if running else .78 if attacking else .80
            _, arms = self.fit(turn, position, weight, contest)
            excess = max(0., max(flex for _, flex in arms) - .88)
            if excess:
                _, arms = self.fit(turn, position, weight / (1. + .75 * excess), contest)
        return self.pack_arms(pose, arms)


def build_fast_time_warp(source_keys, group):
    """Build a monotonic time remapping for the authored fastA..E + fastendA sections.

    Returns (spans, src_to_dst, dst_to_src) where spans is [(name, s0, s1, d0, d1, warped), ...].
    Sections before fasta stay at their original times; sections after fastenda shift by the
    exact cumulative added duration so every channel and text key stays aligned.
    """
    key_map = {lbl.removeprefix(group + ': '): t for t, lbl in source_keys}
    order = ('fasta', 'fastb', 'fastc', 'fastd', 'faste', 'fastenda')
    spans = []
    shift = 0.0
    for name in order:
        s0, s1 = key_map[f'{name} start'], key_map[f'{name} stop']
        target_dur = FAST_TARGETS[name]
        d0 = s0 + shift
        d1 = d0 + target_dur
        warped = name != 'fastenda'
        spans.append((name, s0, s1, d0, d1, warped))
        shift += target_dur - (s1 - s0)

    def src_to_dst(t):
        t = float(t)
        if t < spans[0][1]:
            return t
        for _, s0, s1, d0, d1, warped in spans:
            if t <= s1 + 1e-6:
                if t <= s0:
                    return d0 - (s0 - t)
                u_src = (t - s0) / max(1e-6, s1 - s0)
                u_dst = inv_warp_weight_u(u_src) if warped else u_src
                return d0 + u_dst * (d1 - d0)
        return t + shift

    def dst_to_src(td):
        td = float(td)
        if td < spans[0][3]:
            return td
        for _, s0, s1, d0, d1, warped in spans:
            if td <= d1 + 1e-6:
                if td <= d0:
                    return s0 - (d0 - td)
                u_dst = (td - d0) / max(1e-6, d1 - d0)
                u_src = warp_weight_u(u_dst) if warped else u_dst
                return s0 + u_src * (s1 - s0)
        return td - shift

    return spans, src_to_dst, dst_to_src


def retime_interval(time, start, end, duration):
    length = end - start
    return time + (duration - length) * np.clip((time - start) / length, 0., 1.)


def convert(source, target, retarget):
    stream = load(source)
    group = source.stem
    marks = keys(stream)
    channels = tracks(source)
    original_times = np.unique(np.concatenate((channels[ITEM][0][:, 0], marks.keys['f0'])))
    # Densify the moving intervals so interpolation between local bone keys
    # keeps the left palm close to its rigid item-space target as well.
    midpoints = (original_times[:-1] + original_times[1:]) * .5
    src_times = np.sort(np.r_[original_times, midpoints[np.diff(original_times) > .022]])
    attacking = group == 'weapononehand'
    falling = group == 'brutal_fallattack'
    running = group.startswith('run')
    sprint_start = Pose(retarget.rig, channels, 0.).world(ITEM) if group == 'brutal_sprint1h' else None
    windup_sections = []
    single_hand_sections = {}
    slashright_section = None
    equip_orig_stop = None
    sheath_start = sheath_end = None
    fast_src_ranges = []
    src_to_dst = lambda t: float(t)
    dst_to_src = lambda t: float(t)
    swing_start = 0. if falling else np.inf
    source_keys = [(float(time), str(label)) for time, label in marks.keys]
    if group == 'weapononehand':
        charged = {}
        for attack in ('chop', 'thrust', *WINDUPS):
            prefix = f'{group}: {attack} '
            charged[attack] = {label.removeprefix(prefix): float(time)
                               for time, labels in marks.keys for label in labels.splitlines()
                               if label.startswith(prefix)}
        swing_start = charged['chop']['start']
        windup_sections = [(charged[attack], *bias) for attack, bias in WINDUPS.items()]
        single_hand_sections = {attack: charged[attack] for attack in ('chop', 'thrust')}
        slashright_section = charged['slashright']
        sword = retarget.empty is not None
        equip_attach, equip_catch, equip_stop = SWORD_DRAW_TIMING if sword else EQUIP_TIMING
        equip_orig_stop = next(t for t, l in source_keys if l == f'{group}: equip stop')
        # Retime the 5 authored DM fast swings + fastenda with a smooth weight curve.
        fast_spans, fast_forward, fast_reverse = build_fast_time_warp(source_keys, group)
        fast_src_ranges = [(s0, s1) for name, s0, s1, _, _, _ in fast_spans if name != 'fastenda']
        src_to_dst, dst_to_src = fast_forward, fast_reverse
        ignored = {f'{group}: equip attach', f'{group}: equip stop'}
        extra_keys = [(equip_attach, 'equip attach'), (equip_catch, 'equip catch'), (equip_stop, 'equip stop')]
        if sword:
            sheath_start = next(t for t, l in source_keys if l == f'{group}: unequip start')
            sheath_end = next(t for t, l in source_keys if l == f'{group}: unequip stop')
            next_stop = sheath_start + equip_stop
            src_to_dst = lambda t: retime_interval(fast_forward(t), sheath_start, sheath_end, equip_stop)
            dst_to_src = lambda t: fast_reverse(retime_interval(t, sheath_start, next_stop, sheath_end - sheath_start))
            ignored.update((f'{group}: unequip detach', f'{group}: unequip stop'))
            extra_keys += [(next_stop - equip_catch, 'unequip release'),
                           (next_stop - equip_attach, 'unequip detach'), (next_stop, 'unequip stop')]
        # Remap every text key cleanly to the new timeline.
        remapped = [(src_to_dst(t), label) for t, label in source_keys if label not in ignored]
        remapped += [(t, f'{group}: {label}') for t, label in extra_keys]
        source_keys = sorted(remapped, key=lambda key: key[0])
        # Build dense destination times across equip and warped fast sections.
        dst_times = np.array([src_to_dst(t) for t in src_times], float)
        extra_dst = [np.linspace(0.0, equip_stop, int(np.ceil(equip_stop * 60)) + 1)]
        if sword:
            extra_dst.extend(([time for time, _ in extra_keys], sheath_start + extra_dst[0]))
        for _, _, _, d0, d1, _ in fast_spans:
            samples = np.linspace(d0, d1, int(np.ceil((d1 - d0) * 60)) + 1)
            if sword:
                samples += next_stop - sheath_end
            extra_dst.append(samples)
        dst_times = np.unique(np.concatenate([dst_times, *extra_dst]))
        src_times = np.array([dst_to_src(t) for t in dst_times], float)
    else:
        dst_times = src_times.copy()

    paired_guards = {}
    poses = []
    previous = None
    for dst_t, time in zip(dst_times, src_times):
        pose = Pose(retarget.rig, channels, time)
        turn = matrix_quat(pose.world(ITEM)[:3, :3] @ retarget.source_idle[:3, :3].T)
        if previous is not None and previous @ turn < 0:
            turn = -turn
        if equip_orig_stop is not None and time <= equip_orig_stop + 1e-5:
            poses.append(retarget.equip_pose(pose, min(dst_t, equip_stop)))
            previous = turn
            continue
        if sheath_start is not None and sheath_start - 1e-5 <= time <= sheath_end + 1e-5:
            local = np.clip(sheath_start + equip_stop - dst_t, 0., equip_stop)
            poses.append(retarget.sword_transfer_pose(pose, local))
            previous = turn
            continue
        contest = 0.
        if group.startswith('brutal_clinch'):
            contest = 1.
            if group != 'brutal_clinch':
                t = max(0., 1. - time / .45)
                contest = t * t * (3. - 2. * t)
        windup = None
        for section, offset, tilt, follow in windup_sections:
            weight = charge_weight(time, section, .28, section['min hit'], section['hit'] + follow)
            if weight:
                windup = offset, tilt, weight
        slashright_arc = None
        if slashright_section is not None:
            r0 = slashright_section['min hit']
            rh = slashright_section['hit']
            r1 = slashright_section['large follow stop']
            if r0 < time < r1:
                hit_w = smoothstep((time - r0) / 0.16) * smoothstep((r1 - time) / 0.24)
                follow_w = smoothstep((time - (rh - 0.03)) / 0.16) * smoothstep((r1 - time) / 0.26)
                slashright_arc = (hit_w, follow_w)
        fast_swing = any(s0 - 1e-5 <= time <= s1 + 1e-5 for s0, s1 in fast_src_ranges)
        result = retarget.adapt(pose, turn, attacking, contest, time >= swing_start,
                                falling, windup=windup, running=running, sprint_start=sprint_start,
                                slashright_arc=slashright_arc, fast_swing=fast_swing)
        for attack, section in single_hand_sections.items():
            recover, end = section['hit'] + .25, section['large follow stop']
            if abs(time - section['start']) < 1e-6:
                paired_guards[attack] = result
            elif recover < time <= end:
                # Regrip the drawn guard as the sword settles, without fitting a
                # second pair of arms to the changing wrist during recovery.
                result = paired_guards[attack].copy()
            weight = charge_weight(time, section, .24, recover, end)
            if weight:
                if attack == 'thrust':
                    blend_thrust(pose, result, section, weight)
                else:
                    blend_with_sword(pose, result, weight, sword_grip=False)
        poses.append(result)
        previous = turn
    final_stop = float(dst_times[-1])
    names = [extra.string_data for extra in stream.root.extra_datas if isinstance(extra, nif.NiStringExtraData)]
    for name, controller in zip(names, stream.root.controllers):
        controller.stop_time = final_stop
        if name not in poses[0]:
            # Remap timestamps on all non-arm channels too so their stop_time matches.
            for field in ('rotations', 'translations', 'scales'):
                data = getattr(controller.data, field, None)
                if data is not None and len(data.keys):
                    remapped_keys = data.keys.copy()
                    remapped_keys[:, 0] = [src_to_dst(t) for t in remapped_keys[:, 0]]
                    data.keys = remapped_keys.astype(np.float32)
            continue
        # Adjacent keys use the same quaternion hemisphere, including the
        # small gaps between independently played attack sections.
        rotations = [matrix_quat(pose[name][:3, :3]) for pose in poses]
        for index in range(1, len(rotations)):
            if rotations[index - 1] @ rotations[index] < 0:
                rotations[index] *= -1
        controller.data.rotations.keys = np.column_stack((dst_times, rotations)).astype(np.float32)
        controller.data.translations.keys = np.column_stack((dst_times, [p[name][:3, 3] for p in poses])).astype(np.float32)
    if not set(poses[0]).issubset(names):
        raise ValueError(f'Missing arm channels in {source.name}')
    if group in ('weapononehand', 'brutal_fallattack'):
        source_keys.append((0., group + ': brutal paired grip'))
        source_keys.sort(key=lambda key: key[0])
    yaml = source.with_suffix('.yaml')
    marks.keys = np.array([(time, label.replace(group + ':', target.stem + ':', 1))
                           for time, label in source_keys], dtype=marks.keys.dtype)
    stream.save(target)
    if yaml.is_file():
        # Rename both phase-keyed rules ('group:phase') and whole-group gait rules ('group').
        yaml_text = (yaml.read_text()
                     .replace(f"'{group}:", f"'{target.stem}:")
                     .replace(f"'{group}'", f"'{target.stem}'"))
        if group == 'weapononehand':
            # Crisp yet smooth blend-in for over-the-back draw and heavy fast combo.
            for key, dur in (('equip start', '0.12'), ('fasta start', '0.14'), ('fastb start', '0.14'),
                             ('fastc start', '0.14'), ('fastd start', '0.14'), ('faste start', '0.14'),
                             ('fastenda start', '0.14')):
                yaml_text = re.sub(
                    rf"(- from: '\*'\n\s+to: '{re.escape(target.stem)}:{re.escape(key)}'\n\s+easing: linear\n\s+duration:) [^\n]+",
                    rf'\g<1> {dur}', yaml_text)
        elif group == 'idle1h':
            yaml_text = re.sub(
                rf"(- from: '\*'\n\s+to: '{re.escape(target.stem)}'\n\s+easing: linear\n\s+duration:) [^\n]+",
                r'\g<1> 0.24', yaml_text)
        target.with_suffix('.yaml').write_text(yaml_text)


def build(arms, clinch, native_skeleton, native_animation):
    folder = arms / 'Animations/brutal_fp'
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    idle = Pose(rig, tracks(folder / 'idle1h.kf'), 0.)
    empty = Pose(rig, tracks(folder / 'idle.kf'), 0.)
    native = Pose(Rig(native_skeleton), tracks(native_animation), native_starts(native_animation)['idle2c: start'])
    sources = sorted(path for path in folder.glob('*1h.kf')
                     if path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')))
    groups = {'2c': 'weapontwohand', '2b': 'blunttwohand'}
    guards = ('brutal_block', 'brutal_blockhit1', 'brutal_blockhit2', 'brutal_blockhit3', 'brutal_parrybreak')
    contest = ('brutal_clinch', 'brutal_clinch_win', 'brutal_clinch_lose')
    kicks = ('brutal_kick1h', 'brutal_kick1h_low')
    grips = {}
    for family, weapon_group in groups.items():
        blunt = family == '2b'
        output = folder.with_name('brutal_fp_paired_blunt' if blunt else 'brutal_fp_paired')
        output.mkdir()
        retarget = Retarget(rig, idle, native, blunt, None if blunt else empty)
        convert(folder / 'weapononehand.kf', output / (weapon_group + '.kf'), retarget)
        for path in sources:
            convert(path, output / (path.stem[:-2] + family + '.kf'), retarget)
        # The execution uses the base sword's single-handed grip.
        for group in (*guards, *kicks, 'brutal_fallattack'):
            path = folder / (group + '.kf')
            convert(path, output / path.name, retarget)
        for group in contest:
            path = clinch / 'Animations/brutal_fp' / (group + '.kf')
            convert(path, output / path.name, retarget)
        grips[family] = [grasp.grip.tolist() for grasp in retarget.grasps]
    (arms / 'two-hand-viewmodel-import.json').write_text(json.dumps(dict(
        groups=groups, movement=[p.stem for p in sources], guard=guards, clinch=contest, grips=grips),
        indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arms', type=Path, required=True)
    parser.add_argument('--clinch', type=Path, required=True)
    parser.add_argument('--native-skeleton', type=Path, required=True)
    parser.add_argument('--native-animation', type=Path, required=True)
    args = parser.parse_args()
    build(args.arms, args.clinch, args.native_skeleton, args.native_animation)


if __name__ == '__main__':
    main()
