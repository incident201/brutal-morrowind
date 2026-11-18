#!/usr/bin/env python3
"""First-person spear thrusts, using native finger curls and the DM staff's release/regrip."""
import argparse
from functools import lru_cache
import hashlib
import json
import re
from pathlib import Path
from typing import NamedTuple
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


def steer_arm(start, end, grasp, hand, w, relax=0., parent=None, fixed_socket=False, shoulder_phase=None):
    """Move a bent arm between two poses, meeting the hand target at both bone lengths."""
    clavicle, upper, fore, wrist = grasp.names
    C, U, F, H = [blend(start.world(n), end.world(n), w) for n in grasp.names]
    if fixed_socket:
        if shoulder_phase is not None:
            C = blend(start.world(clavicle), end.world(clavicle), shoulder_phase)
        U[:3, 3] = (C @ start.local(upper))[:3, 3]
    a, b = grasp.segments
    delta = hand[:3, 3] - U[:3, 3]
    d = np.linalg.norm(delta)
    axis = unit(delta)
    shifted = axis * max(0., d - (a + b - .35))
    if fixed_socket and np.any(shifted):
        hand = hand.copy()
        hand[:3, 3] -= shifted
    else:
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


class ThrustPose(NamedTuple):
    # Wrist position and shaft direction in the first-person camera frame.
    wrist: tuple[float, float, float]
    direction: tuple[float, float, float]
    roll: float
    shoulder: float
    elbow: float | None = None
    support: bool = False


class Spear:
    def __init__(self, rig, source, marks, native, fist, profile, single):
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
        left.grip[1, 3] = right.grip[1, 3] + 31.
        left.pole = unit(np.array((-.8, -.58, .15)))
        left.wrist_bias = 6.
        left.grip = matrix(turn_about((0, 1, 0), -24.), np.zeros(3)) @ left.grip
        for side, g in zip(SIDES, self.grasps):
            original = g.grip.copy()
            close_hand(g, profile, side)
            if side == 'L':
                g.grip = blend(original, g.grip, .82)
            for name, pose in g.fingers.items():
                weight = .32 if 'Finger0' in name else .27
                g.fingers[name] = blend(pose, fist.local(name), weight)
        self.carrying_arm = Grasp(native, 'R', False)
        self.carrying_arm.wrist_bias = 2.8
        # Body parts meet at the native bind frame of the forearm and hand.
        # The wide-weapon idle already contains a large wrist rotation.
        self.bind_wrist = Pose(native.rig, {}, 0.).local(right.names[-1])[:3, :3]
        self.single_grip = inverse(single.world(ITEM)) @ single.world(right.names[-1])
        self.single_fingers = {n: blend(single.local(n), fist.local(n), .78 if 'Finger0' in n else .19)
                               for n in right.fingers}
        # Right thumb through little finger. Flex the native joints and
        # oppose the thumb across the wrap. These angles seat all five distal
        # pads on the approximately 0.7-unit shaft of the owned weapon meshes.
        # Values are MCP, PIP, DIP flexion and base abduction in local degrees.
        curls = [(-4., -23., -7., 48.), (-30., 22., 19., 30.), (-11., 16., -12., 17.),
                 (18., 1., 7., -18.), (32., -30., -24., -30.)]
        for digit, (base, middle, tip, spread) in enumerate(curls):
            for segment, flexion in zip(('', '1', '2'), (base, middle, tip)):
                joint = self.single_fingers[bone('R', f'Finger{digit}{segment}')]
                turn = turn_about((0, 0, 1), flexion)
                if not segment:
                    turn = turn_about((0, 1, 0), spread) @ turn
                joint[:3, :3] = joint[:3, :3] @ turn
        self.names = [*source]
        self.open_hand = Pose(rig, source, self.release + 49. / 33.)
        self.guard = self.strike(0., 0., 0., 0.)

    def strike(self, travel, push, across, height, release=0.):
        # Author the screen silhouette in camera space. During a full lunge the
        # rear hand carries the pole while the other hand opens and falls back.
        freed = ease(release)
        diagonal = unit(np.array((-.774, .554, -.31)))
        forward = unit(np.array((-9. - across*.70, 4. - height*.7 - 7.*freed, -49.)))
        axis = unit(lerp(diagonal, forward, push) + np.array((-.012*across, -.010*height, 0.))*(1.-push))
        rear = np.array((17.4 + across - 4.*push, -20.3 + height + 6.*push + 16.*freed,
                         -27.4 + 16.*push)) + travel * axis
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

    @lru_cache(maxsize=64)
    def keypose(self, pose):
        guard = LocalPose(self.rig, self.guard)
        if pose is None:
            return guard
        right, left = self.grasps
        rest = guard.world(ITEM)
        axis = self.camera[:3, :3] @ unit(np.array(pose.direction))
        rotation = swing(rest[:3, 1], axis) @ rest[:3, :3] @ turn_about((0, 1, 0), pose.roll)
        rear = (self.camera @ np.r_[pose.wrist, 1.])[:3]
        right_grip = inverse(rest) @ guard.world(right.names[-1])
        # Morrowind's one-handed thrust curls each finger around the shaft.
        # Retain the donor curl and radial anchor while changing carry position.
        # Only the station along the haft changes when the two-handed grip opens.
        station = right_grip[1, 3] + 28.
        right_grip = self.single_grip.copy()
        # Let the haft sit at a shallow diagonal across the closed palm.
        # Pivot at the contact so the fingers stay wrapped as the arm extends.
        palm = right_grip[:3, :3] @ right.palm_along
        tangent = unit(palm - palm[1]*np.array((0., 1., 0.)))
        slope = np.radians(42.)
        turn = swing(palm, np.cos(slope)*tangent + np.array((0., np.sin(slope), 0.)))
        # Centre the diagonal through the native curled finger arcs.
        contact = np.array((-1.12, station-self.single_grip[1, 3], .46))
        right_grip = matrix(turn @ right_grip[:3, :3], contact + turn @ right_grip[:3, 3])
        item = matrix(rotation, rear - rotation @ right_grip[:3, 3])
        result = self.guard.copy()
        # Measure the contact frames on the actual skinned guard. Transport the
        # same radial contacts as the guiding hand slides lengthwise on the haft.
        for side, grasp in zip(SIDES, self.grasps):
            grip = inverse(rest) @ guard.world(grasp.names[-1])
            if side == 'R':
                grip = right_grip
            if side == 'L' and pose.support:
                # The shaft rolls inside the loose supporting palm during
                # the pickup. Only the rear palm follows the weapon's bank.
                unroll = matrix(turn_about((0, 1, 0), -pose.roll), np.zeros(3))
                grip = unroll @ grip
                grip[1, 3] = station + 10.
            hand = item @ grip
            free = side == 'L' and not pose.support
            if free:
                # The open hand counterbalances the carrying arm. Use a
                # relaxed curve below the line of sight, visible on extension.
                lead = np.clip(pose.shoulder, 0., 1.)
                pocket = (-17. - 3.*lead, -20. - 5.*lead,
                          -21. + 4.*lead)
                hand[:3, 3] = (self.camera @ np.r_[pocket, 1.])[:3]
            drive = pose.shoulder
            # The shoulder socket moves on the collarbone arc. Both ends of
            # the upper arm stay connected; the wrist cannot drag the shoulder
            # out into the middle of the first-person view.
            clavicle = guard.world(grasp.names[0]).copy()
            socket = guard.world(grasp.names[1])[:3, 3]-clavicle[:3, 3]
            # Carry the high elbow with the raised shoulder. The socket
            # still travels on the same rigid collarbone.
            high = np.clip(-pose.direction[1]*4., 0., 1.)
            reach = self.camera[:3, :3] @ np.array((.14, .08+.60*high, -1.))
            swing_to = unit(lerp(unit(socket), reach, (.87*drive if side == 'R' else .34*pose.support)))
            arc = swing(socket, swing_to)
            clavicle[:3, :3] = arc @ clavicle[:3, :3]
            shoulder = clavicle[:3, 3] + arc @ socket
            shift = shoulder - grasp.shoulder
            offset = matrix(np.eye(3), shift)
            hint = (29., -28., -13.) if side == 'R' else (-23., -26., -14.)
            elbow = (self.camera @ np.r_[hint, 1.])[:3] - shift
            # Rotating the staff around its long axis changes which side of
            # the closed hand faces the camera. Let the elbow settle below
            # the wrist across the full one-handed range.
            swivels = np.radians(np.linspace(-145., 145., 19)) if side == 'R' else np.array((-.18, 0., .18))
            if side == 'R' and pose.elbow is not None:
                follow = ease((drive-.27)/.61)
                elbow = lerp(elbow, hand[:3, 3]-axis*grasp.segments[1]-shift, follow)
                swivels = np.radians((1.-follow)*pose.elbow + np.array((-8., 0., 8.)))
            solver = self.carrying_arm if side == 'R' else grasp
            arm, _ = solver.fit(inverse(offset) @ hand @ inverse(solver.grip), elbow, swivels)
            if side == 'R':
                collar, upper, forearm, wrist = arm
                # Match forearm pronation to the native wrist seam while
                # preserving the elbow and both segment lengths.
                rest_fore = wrist[:3, :3] @ self.bind_wrist.T
                forearm[:3, :3] = swing(rest_fore[:, 0], wrist[:3, 3]-forearm[:3, 3]) @ rest_fore
            if free:
                open_hand = self.open_hand.world(grasp.names[-1]).copy()
                open_hand[:3, 3] = arm[-1][:3, 3]
                arm = (*arm[:-1], relaxed_wrist(arm[-2], open_hand, grasp, .82))
            arm = (inverse(offset) @ clavicle, *arm[1:])
            parent = guard.world(self.rig.parents[grasp.names[0]])
            for name, transform in zip(grasp.names, arm):
                world = offset @ transform
                result[name] = inverse(parent) @ world
                parent = world
            if free:
                for n in grasp.fingers:
                    result[n] = blend(self.guard[n], self.open_hand.local(n), .53)
        result.update(self.single_fingers)
        result[ITEM] = inverse(right_grip)
        return LocalPose(self.rig, result)

    def beat(self, t, path):
        for (start, a), (end, b) in zip(path, path[1:]):
            if t <= end:
                if a == b:
                    return self.keypose(a).transforms
                first, last = self.keypose(a), self.keypose(b)
                phase = float(np.clip((t-start)/(end-start), 0., 1.))
                w = ease(phase)
                # The clavicle initiates the reach, followed by the elbow and
                # finally the wrist. Overlap the free hand in the other phase.
                shoulder = ease(phase + .55*phase*(1.-phase))
                support = ease(phase - .55*phase*(1.-phase))
                result = {n: blend(first.local(n), last.local(n), w) for n in self.names}
                right, left = self.grasps
                rear = blend(first.world(right.names[-1]), last.world(right.names[-1]), w)
                item = rear @ result[ITEM]
                result.update(steer_arm(first, last, right, rear, w, fixed_socket=True,
                                        shoulder_phase=shoulder))
                h = left.names[-1]
                hand = blend(first.world(h), last.world(h), support)
                if (a is None or a.support) and (b is None or b.support):
                    # A sliding guiding palm can stay on the haft between
                    # two contacts. Once it releases, interpolate in body
                    # space so that a rolling shaft cannot fling it across.
                    hand = item @ blend(inverse(first.world(ITEM)) @ first.world(h),
                                        inverse(last.world(ITEM)) @ last.world(h), w)
                    collar, upper = left.names[:2]
                    socket = (blend(first.world(collar), last.world(collar), support) @ first.local(upper))[:3, 3]
                    offset = hand[:3, 3]-socket
                    along = item[:3, 1]
                    axial = offset @ along
                    radial = offset-axial*along
                    limit = np.sqrt(max(0., (sum(left.segments)-.45)**2 - radial @ radial))
                    hand[:3, 3] += along*(np.clip(axial, -limit, limit)-axial)
                result.update(steer_arm(first, last, left, hand, support, fixed_socket=True))
                return result
        return self.keypose(path[-1][1]).transforms

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
    recovery = Pose(rig, channels, release + 43. / 33.).world(ITEM)
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    controls = dict(zip(names, stream.root.controllers))
    for i, t in enumerate(channels[grasp.names[-1]][0][:, 0]):
        frame = (float(t) - release) * 33.
        if 43. < frame < 56.:
            # The returning right wrist rolls the mesh 64 degrees in one
            # source frame, inside the still closed supporting palm. Keep the
            # shaft's direction and mounting point, then let its axial turn
            # catch up while the source regrip has the fingers open.
            pose = Pose(rig, channels, t)
            item = pose.world(ITEM)
            follow = swing(recovery[:3, 1], item[:3, 1]) @ recovery[:3, :3]
            difference = item[:3, :3].T @ follow
            roll = np.degrees(np.arctan2(difference[0, 2], difference[0, 0]))
            weight = 1. - ease((frame-45.)/11.)
            mounted = pose.local(ITEM)[:3, :3] @ turn_about((0, 1, 0), roll*weight)
            keys = controls[ITEM].data.rotations.keys
            q = matrix_quat(mounted)
            keys[i, 1:] = q if keys[i-1, 1:] @ q >= 0. else -q
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
    single = Pose(native_rig, native_tracks, starts['weapononehand: thrust hit'])
    spear = Spear(rig, channels, marks, vanilla, fist, profile, single)
    guard = None
    # Move the blade and wrist together along a fixed line in camera space.
    def along(chamber, distance, drive):
        wrist = np.array(chamber.wrist) + unit(np.array(chamber.direction))*distance
        return chamber._replace(wrist=tuple(wrist), shoulder=drive)

    guide = ThrustPose((13., -8., -22.), (-.43, .15, -.89), 126., .18, 0., True)
    deep = ThrustPose((16., -7., -14.), (-.26, .095, -.961), 162., -.17, -8.)
    single = deep
    extended = along(single, 25., .94)
    # Contact decelerates over a little axial give. The shoulder carries on
    # as the wrist settles, then the hand peels back into the two-hand catch.
    charged = [(0., deep), (.247, extended), (.34, along(single, 24.72, .99)),
               (.405, along(single, 24.48, .92)), (.64, along(single, 12., .30)), (.89, guard)]
    segments = []
    labels = []
    thrust_sections = set()
    body_paths = {}

    def section(name, start, length, motion, events, body=()):
        if body:
            body_paths[start] = body
        thrust_sections.add(start)
        segments.append((start, start + length, motion))
        labels.extend((start + t, 'weapononehand: ' + name + ' ' + event) for t, event in events)

    start, maximum, release = (marks['thrust ' + key] for key in ('start', 'max attack', 'min hit'))
    preparation = [(0., guard), (.20, guide), (.50, deep), (maximum-start, deep)]
    section('thrust', start, maximum - start, lambda t: spear.beat(t, preparation),
            [(0., 'start'), (marks['thrust min attack'] - start, 'min attack'), (maximum - start, 'max attack')],
            [(0., (0., 0., 0.)), (.43, (2., -3., -1.5)), (maximum-start, (2., -3., -1.5))])
    end = .9
    section('thrust', release, end, lambda t: spear.beat(t, charged),
            [(0., 'min hit'), (0., 'plant'), (.242, 'hit'), (.27, 'weight forward'), (.62, 'chain')] +
            [(t, length + ' follow ' + edge) for t, edge in ((.243, 'start'), (end, 'stop'))
             for length in ('small', 'medium', 'large')],
            [(0., (2., -3., -1.5)), (.13, (1., 1.8, -3.6)), (.285, (-2.3, 5., -2.)),
             (.44, (-1.8, 4.2, -.5)), (end, (0., 0., 0.))])
    hold = marks['thrust hold start']
    held_pose = spear.keypose(deep).transforms
    section('thrust hold', hold, .22, lambda t: held_pose,
            [(0., 'start'), (.02, 'loop start'), (.22, 'loop stop'), (.22, 'stop')],
            [(0., (2., -3., -1.5)), (.22, (2., -3., -1.5))])
    labels.extend([(hold + .02, 'weapononehand: loop start'), (hold + .22, 'weapononehand: loop stop')])
    # Draw, settle, drive, hold at contact, then recover. The support hand
    # stays clear between strokes; fastend is the separate return to guard.
    # New sections live past the source clips so longer holds cannot overlap
    # the next motion or borrow its last frame.
    attacks = [
        # duration, back of draw, end of drive, end of fixation, travel
        ('a', .64, .16, .33, .435, 23.,
         ThrustPose((15., -7.2, -15.), (-.32, .095, -.944), 151., -.08, -5.)),
        ('b', .71, .19, .37, .493, 25.,
         ThrustPose((19., -14.4, -13.7), (-.19, .092, -.978), 115., -.10, -32.)),
        ('c', .71, .18, .36, .485, 24.3,
         ThrustPose((16., 6., -11.), (-.30, -.175, -.9377), 174., -.06, -18.)),
        ('d', .78, .225, .446, .580, 30.,
         ThrustPose((20., -2., -11.), (-.58, -.055, -.8128), 148., -.13, -5.)),
    ]
    # Clear the struck line before the next hand changes height. The next
    # attack then starts from the same one-handed carry as the previous exit.
    ready = ThrustPose((12., -6.5, -27.5), (-.25, .07, -.966), 150., .41, 4.)
    cursor = max(float(t) for t, _ in text.keys) + .2
    for letter, duration, draw, impact, fixed, distance, chamber in attacks:
        point = along(chamber, distance, .85 if letter != 'd' else 1.)
        finish = ready
        # A slight change of pressure at contact keeps the hand alive while
        # the weapon remains visibly extended before the recoil.
        settle = along(chamber, distance-.32, point.shoulder+.04)
        recoil = along(chamber, distance-13., .55)
        path = [(0., ready), (draw, chamber), (impact, point),
                (fixed, settle), (fixed+.098, recoil), (duration, finish)]
        sections = [('fast'+letter, path, 0.)]
        if letter == 'a':
            # The first release and withdrawal are one gesture. Going via an
            # already extended ready pose here would look like a false hit.
            pickup = .085
            entry = [(0., guard)] + [(pickup+t, pose) for t,pose in path[1:]]
            loop_chamber = ThrustPose((15., -7.2, -11.), (-.40, .095, -.9116), 151., -.10, -5.)
            loop_peak = along(loop_chamber, 29.4, .85)
            loop_draw = draw+.075
            loop = [(0., ready), (loop_draw, loop_chamber), (impact+.075, loop_peak),
                    (fixed+.075, along(loop_chamber, 29.1, .89)),
                    (fixed+.173, along(loop_chamber, 13.5, .45)), (duration+.075, ready)]
            sections = [('fasta', entry, pickup), ('fastloopa', loop, .075)]
        for name, motion, pickup in sections:
            length = duration+pickup
            section(name, cursor, length, lambda t, p=motion: spear.beat(t, p),
                    [(0., 'start'), (pickup+draw-.038, 'plant'), (pickup+draw, 'swing'),
                     (pickup+impact+.035, 'weight forward'),
                     (pickup+impact-.005, 'thrust hit'), (length, 'stop')],
                    [(0., (0., 0., 0.)), (pickup+draw, (1.1, -1.6, -.55)),
                     (pickup+impact-.055, (-.3, 1.4, -2.35)),
                     (pickup+impact+.042, (-1.55, 3.1, -1.45)), (length, (0., 0., 0.))])
            cursor += length + .06
        recovery = [(0., finish), (.035, finish), (.29, guard)]
        section('fastend' + letter, cursor, .29, lambda t, p=recovery: spear.beat(t, p),
                [(0., 'start'), (.29, 'stop')])
        cursor += .35

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
        rate = 120 if begin in thrust_sections else 60
        times = np.linspace(begin, stop, max(2, int(np.ceil((stop - begin) * rate)) + 1))
        for t in times:
            elapsed = float(t-begin)
            frame = fn(elapsed)
            if begin in body_paths:
                path = body_paths[begin]
                for (t0, offset0), (t1, offset1) in zip(path, path[1:]):
                    if elapsed <= t1 + 1e-6:
                        offset = lerp(np.array(offset0), np.array(offset1), ease((elapsed-t0)/(t1-t0)))
                        carrier = frame['Brutal Camera Carrier'].copy()
                        carrier[:3, 3] += offset
                        frame = {**frame, 'Brutal Camera Carrier': carrier}
                        break
            samples[float(t)] = frame
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
    # The staff's 200 ms cross-fade consumes the short spear withdrawal.
    # These clips already meet at an authored ready pose. Keep a small blend
    # for a fresh click arriving partway through the return to guard.
    rules = source.with_suffix('.yaml').read_text()
    rules = re.sub(r"(to: 'weapononehand:fast(?:end)?[abcd] start'\s+easing: linear\s+duration: )0\.2[0-9]*",
                   r"\g<1>0.045", rules)
    rules += "  - from: '*'\n    to: 'weapononehand:fastloopa start'\n    easing: linear\n    duration: 0.045\n"
    (target / 'weapononehand.yaml').write_text(rules)
    for path in sorted(folder.glob('*1h.kf')):
        if path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            movement_overlay(spear, path, target)
    manifest_path = arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())
    for path in (source, *sorted(target.iterdir())):
        manifest['outputs'][path.relative_to(arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['spear'] = {'charged': 'thrust', 'quick': [row[0] for row in attacks],
                         'base': 'brutal_fp_staff', 'native_grip': 'idle2w',
                         'one_hand_grip': 'weapononehand: thrust hit'}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('arms', 'native-skeleton', 'native-animation'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    build(args.arms, args.native_skeleton, args.native_animation)
