#!/usr/bin/env python3
"""Retarget the imported first-person sword motion around Morrowind's paired grips."""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_staff_mesh_grip import Rig, matrix, nif, slerp
from audit_viewmodel_continuity import tracks
from retarget_kf import cross, matrix_quat, norm, quat_matrix


ITEM = 'Weapon Bone'
ROOT = 'ModelRoot'
SIDES = ('R', 'L')
ARM = ('Clavicle', 'UpperArm', 'Forearm', 'Hand')
SWIVELS = np.linspace(-1.2, 1.2, 17)
# Offset from the paired grip, outward tilt, and time to finish easing out after contact.
WINDUPS = {'neutral': ((2., 9., -23.), 0., .15), 'slashright': ((16., 7., -7.), 24., 0.)}


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


def charge_weight(time, marks, draw_time, return_start, return_stop):
    if marks['hold start'] <= time <= marks['hold stop']:
        return 1.
    start = marks['start']
    if not start < time < return_stop:
        return 0.
    return smoothstep((time - start) / draw_time) * smoothstep((return_stop - time) / (return_stop - return_start))


def blend_with_sword(pose, paired, weight):
    # Keep the closed right fist and weapon socket. Let the authored arm motion
    # carry the swing, with the left hand opening and returning to the paired grip.
    for name, transform in paired.items():
        if name == ITEM or name.startswith(bone('R', 'Finger')):
            continue
        sword = pose.local(name)
        if weight >= 1.:
            paired[name] = sword
        else:
            first, second = (matrix_quat(m[:3, :3]) for m in (transform, sword))
            rotation = quat_matrix(slerp(first, second, weight)[[1, 2, 3, 0]])
            paired[name] = matrix(rotation, lerp(transform[:3, 3], sword[:3, 3], weight))


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


class Grasp:
    def __init__(self, native, side, blunt):
        self.names = [bone(side, joint) for joint in ARM]
        self.clavicle, upper, fore, hand = map(native.world, self.names)
        self.shoulder, self.elbow, self.wrist = upper[:3, 3], fore[:3, 3], hand[:3, 3]
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

    def fit(self, item, elbow_hint=None, swivels=SWIVELS, smooth_bend=False):
        hand = item @ self.grip
        wrist = hand[:3, 3]
        delta = wrist - self.shoulder
        distance = norm(delta)
        axis = delta / distance
        upper_length, fore_length = self.segments
        projection = (distance ** 2 + upper_length ** 2 - fore_length ** 2) / (2 * distance)
        height = np.sqrt(max(0., upper_length ** 2 - projection ** 2))
        base = self.shoulder + axis * projection
        guide = self.pole if elbow_hint is None else elbow_hint - self.shoulder
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
            elbow = base + height * (np.cos(angle) * pole + np.sin(angle) * tangent)
            fore_axis = unit(wrist - elbow)
            # Let axial forearm rotation absorb palm roll while respecting the
            # bent elbow. The angle between forearm and native palm stays measurable.
            flex = np.arccos(np.clip(fore_axis @ desired_fore[:, 0], -1., 1.))
            palm_flex = np.arccos(np.clip(fore_axis @ palm_along, -1., 1.))
            normal = cross(elbow - self.shoulder, wrist - elbow)
            upper_rotation = basis(elbow - self.shoulder, normal) @ self.upper_frame
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
        return (self.clavicle, matrix(upper_rotation, self.shoulder), matrix(fore_rotation, elbow), hand), max(
            flex, palm_flex - .18)


def keys(stream):
    return next(extra for extra in stream.root.extra_datas if isinstance(extra, nif.NiTextKeyExtraData))


def load(path):
    stream = nif.NiStream()
    stream.load(path)
    return stream


def native_starts(path):
    return {key.lower(): float(time) for time, text in keys(load(path)).keys for key in text.splitlines() if key}


class Retarget:
    def __init__(self, rig, sword_idle, native, blunt):
        self.rig = rig
        self.source_idle = sword_idle.world(ITEM)
        self.idle = native.world(ITEM).copy()
        self.idle[:3, 3] += (-4., 9., 4.)
        self.idle[:3, :3] = turn_about((1, 0, 0), -18) @ self.idle[:3, :3]
        self.stance_offset = matrix_quat(self.source_idle[:3, :3].T @ self.idle[:3, :3])
        self.grasps = [Grasp(native, side, blunt) for side in SIDES]
        self.item_mount = inverse(self.grasps[0].grip)

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

    def fit_item(self, item, swing=False, one_hand=False):
        grasps = self.grasps[:1] if one_hand else self.grasps
        for _ in range(4):
            for grasp in grasps:
                wrist = (item @ grasp.grip)[:3, 3]
                delta = wrist - grasp.shoulder
                length = norm(delta)
                reachable = np.clip(length, 11. if swing else 8., sum(grasp.segments) - 1.3)
                item[:3, 3] += delta * (reachable / length - 1.)
        arms = [grasp.fit(item, smooth_bend=swing) for grasp in grasps]
        return item, arms

    def swing_item(self, source, delta, turn, falling, windup):
        # The paired guard is higher than the sword's. As the hands leave it,
        # remove most of that offset so the authored swing crosses the view.
        angle = 2. * np.arccos(np.clip(abs(turn[0]), 0., 1.))
        motion = min(1., np.hypot(norm(delta) / 34., angle / 1.1))
        weight = motion * motion * (3. - 2. * motion)
        offset = slerp(self.stance_offset, np.array((1., 0., 0., 0.)), .72 * weight)
        rotation = source[:3, :3] @ quat_matrix(offset[[1, 2, 3, 0]])
        position = self.idle[:3, 3] + delta * (1., .88, .95) + weight * np.array((0., 0., -22.))
        position[1] = max(13., position[1])
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
        return matrix(rotation, position)

    def sprint_item(self, source, sword_start):
        # Carry the long weapon lower on the right. Follow the sword's cycle
        # with a short stroke so the tip stays clear of the centre of the view.
        travel = .20
        turn = matrix_quat(source[:3, :3] @ sword_start[:3, :3].T)
        carry = turn_about((0, 0, 1), -16) @ turn_about((1, 0, 0), 20) @ sword_start[:3, :3]
        position = np.array((23., 24., 95.)) + travel * (source[:3, 3] - sword_start[:3, 3])
        return matrix(scale_turn(turn, travel) @ carry, position)

    def adapt(self, pose, turn, attacking, contest, swing, falling, *, windup, running, sprint_start):
        source = pose.world(ITEM)
        delta = source[:3, 3] - self.source_idle[:3, 3]
        # Transfer the sword's world-space swing about our different idle
        # orientation. Translation is measured from the same sword idle.
        if sprint_start is not None:
            # Keep the authored free left arm and fingers from the sword sprint.
            _, arms = self.fit_item(self.sprint_item(source, sprint_start), swing=True, one_hand=True)
        elif swing:
            _, arms = self.fit_item(self.swing_item(source, delta, turn, falling, windup), swing=True)
        else:
            travel = (.66, .53, .66) if attacking else (.47, .52, .53)
            position = self.idle[:3, 3] + delta * travel
            position[1] = max(17., position[1])
            weight = .32 if running else .78 if attacking else .80
            _, arms = self.fit(turn, position, weight, contest)
            excess = max(0., max(flex for _, flex in arms) - .88)
            if excess:
                _, arms = self.fit(turn, position, weight / (1. + .75 * excess), contest)
        result = {ITEM: self.item_mount}
        for grasp, (chain, _) in zip(self.grasps, arms):
            parent = pose.world(self.rig.parents[grasp.names[0]])
            for name, transform in zip(grasp.names, chain):
                result[name] = inverse(parent) @ transform
                parent = transform
            result.update(grasp.fingers)
        return result


def convert(source, target, retarget):
    stream = load(source)
    group = source.stem
    marks = keys(stream)
    channels = tracks(source)
    original_times = np.unique(np.concatenate((channels[ITEM][0][:, 0], marks.keys['f0'])))
    # Densify the moving intervals so interpolation between local bone keys
    # keeps the left palm close to its rigid item-space target as well.
    midpoints = (original_times[:-1] + original_times[1:]) * .5
    times = np.sort(np.r_[original_times, midpoints[np.diff(original_times) > .022]])
    attacking = group in ('weapononehand', 'brutal_ground_finisher')
    falling = group == 'brutal_fallattack'
    running = group.startswith('run')
    sprint_start = Pose(retarget.rig, channels, 0.).world(ITEM) if group == 'brutal_sprint1h' else None
    windup_sections = []
    chop = None
    swing_start = 0. if falling else np.inf
    if group == 'weapononehand':
        charged = {}
        for attack in ('chop', *WINDUPS):
            prefix = f'{group}: {attack} '
            charged[attack] = {label.removeprefix(prefix): float(time)
                               for time, labels in marks.keys for label in labels.splitlines()
                               if label.startswith(prefix)}
        chop = charged['chop']
        swing_start = chop['start']
        windup_sections = [(charged[attack], *bias) for attack, bias in WINDUPS.items()]
    chop_guard = None
    poses = []
    previous = None
    for time in times:
        pose = Pose(retarget.rig, channels, time)
        turn = matrix_quat(pose.world(ITEM)[:3, :3] @ retarget.source_idle[:3, :3].T)
        if previous is not None and previous @ turn < 0:
            turn = -turn
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
        result = retarget.adapt(pose, turn, attacking, contest, time >= swing_start,
                                falling, windup=windup, running=running, sprint_start=sprint_start)
        if chop:
            recover, end = chop['hit'] + .25, chop['large follow stop']
            if time == chop['start']:
                chop_guard = result
            elif recover < time <= end:
                # Regrip the drawn guard as the sword settles, without fitting a
                # second pair of arms to the changing wrist during recovery.
                result = chop_guard.copy()
            weight = charge_weight(time, chop, .24, recover, end)
            if weight:
                blend_with_sword(pose, result, weight)
        poses.append(result)
        previous = turn
    names = [extra.string_data for extra in stream.root.extra_datas if isinstance(extra, nif.NiStringExtraData)]
    for name, controller in zip(names, stream.root.controllers):
        if name not in poses[0]:
            continue
        # Adjacent keys use the same quaternion hemisphere, including the
        # small gaps between independently played attack sections.
        rotations = [matrix_quat(pose[name][:3, :3]) for pose in poses]
        for index in range(1, len(rotations)):
            if rotations[index - 1] @ rotations[index] < 0:
                rotations[index] *= -1
        controller.data.rotations.keys = np.column_stack((times, rotations)).astype(np.float32)
        controller.data.translations.keys = np.column_stack((times, [p[name][:3, 3] for p in poses])).astype(np.float32)
    if not set(poses[0]).issubset(names):
        raise ValueError(f'Missing arm channels in {source.name}')
    source_keys = list(marks.keys)
    if group in ('weapononehand', 'brutal_fallattack'):
        source_keys.append((0., group + ': brutal paired grip'))
        source_keys.sort(key=lambda key: key[0])
    yaml = source.with_suffix('.yaml')
    marks.keys = np.array([(time, label.replace(group + ':', target.stem + ':', 1))
                           for time, label in source_keys], dtype=marks.keys.dtype)
    stream.save(target)
    if yaml.is_file():
        target.with_suffix('.yaml').write_text(yaml.read_text().replace(group + ':', target.stem + ':'))


def build(arms, clinch, native_skeleton, native_animation):
    folder = arms / 'Animations/brutal_fp'
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    idle = Pose(rig, tracks(folder / 'idle1h.kf'), 0.)
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
        retarget = Retarget(rig, idle, native, blunt)
        convert(folder / 'weapononehand.kf', output / (weapon_group + '.kf'), retarget)
        for path in sources:
            convert(path, output / (path.stem[:-2] + family + '.kf'), retarget)
        for group in (*guards, *kicks, 'brutal_ground_finisher', 'brutal_fallattack'):
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
