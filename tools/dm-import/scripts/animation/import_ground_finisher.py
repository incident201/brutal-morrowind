#!/usr/bin/env python3
"""Retarget the three ground-impale layers from the user's own game files."""
import argparse
import json
from pathlib import Path

import numpy as np

from calibrate_arm_frames import corrections, load_calibration
from import_dagger_anatomy import FRAME_NODES
from import_native_sword_visual import NativeSword
from import_staff_native_grasp import StaffNativeGrasp
from import_two_hand_viewmodel import Pose, Rig, native_starts, tracks
from import_viewmodel import CAMERA_CARRIER, Viewmodel, frame
from import_weapon_families import LEFT_WEAPON
from native_wrist_profile import lateral_cap


GROUP = 'brutal_ground_finisher'
PHASES = (('in', 'start', 'entry stop'), ('idle', 'strike start', 'strike stop'),
          ('out', 'return start', 'stop'))


class SwordReversal:
    """The Source sword entity plays its own track at events 2149 and 2150."""
    model = 'items/weapons/animations/sword_animations'
    lower = '@sword_updown'
    raise_ = '@sword_downup'
    palm = np.array([0., -1., 0.])

    def __init__(self, vm):
        self.vm = vm
        sword = 'items/weapons/sword_long/sword_long'
        root = vm.dm_world(sword, '@idle', 0.)['sword']
        self.normal = root[:3, :3]
        idle = vm.weapon_idle[:3, :3]
        self.axes = frame(idle @ [0., -1., 0.], idle @ [0., 0., -1.]) @ frame([0, 1, 0], [1, 0, 0]).T
        self.lower_duration = vm.duration(self.model, self.lower)
        self.raise_duration = vm.duration(self.model, self.raise_)

    def apply(self, pose, time, lower_at, raise_at):
        if time <= lower_at or time >= raise_at + self.raise_duration:
            return pose
        if time >= raise_at:
            clip, elapsed = self.raise_, time - raise_at
        else:
            clip = self.lower
            # The original two-frame turn is hard to read at 60 Hz. Ease it
            # across the start of the windup, well before the strike.
            phase = min(1., (time - lower_at) / .16)
            elapsed = self.lower_duration * phase * phase * (3. - 2. * phase)
        item = self.vm.dm_world(self.model, clip, elapsed)['sword']
        mount, offset = pose['Weapon Bone']
        rotation = mount @ self.axes.T @ item[:3, :3] @ self.normal.T @ self.axes
        shift = mount @ self.axes.T @ item[:3, 3] * self.vm.body_scale
        result = dict(pose)
        result['Weapon Bone'] = rotation, offset + mount @ self.palm - rotation @ self.palm + shift
        return result


def build(vm, folder, model, kind, process_pose, reversal=None):
    ready, ready_fade = vm.stance_pose()
    start_pose = ready
    body_prefix = f'@layer_ground_impal_{kind}_'
    standing_eye = vm.body_eye_carrier(body_prefix + 'in', 0.)
    roll_state = {}
    offset, segments, marks, item_events = 0., [], [], {}
    for phase, start_key, stop_key in PHASES:
        clip = f'anm_Ground_Impal_{kind}_{phase}'
        natural = vm.duration(model, clip)
        if phase == 'out':
            head = vm.duration('player_head_mvt', f'@layer_Ground_Impal_{kind}_out')
            settle = (ready, max(ready_fade, head - natural))
        else:
            settle = None
        times, poses, _ = vm.segment(model, clip, duration=natural, roll_state=roll_state,
                                     start_pose=start_pose, return_to=settle, runtime_entry=phase == 'in')
        # MMeyes supplies the dip of the body. The separate head layer only
        # turns the view. Time the body layer to the same arm phase boundaries.
        body = body_prefix + phase
        body_speed = vm.duration('player_animations', body) / natural
        vm.pending[-1]['body_camera'] = body
        fade = vm.sequence_fade(model, clip)
        previous_eye = start_pose[CAMERA_CARRIER][1]
        for time, pose in zip(times, poses):
            eye = vm.body_eye_carrier(body, min(time, natural) * body_speed) - standing_eye
            weight = min(1., time / fade) if fade > 0. else 1.
            pose[CAMERA_CARRIER] = (np.eye(3), (1. - weight) * previous_eye + weight * eye)
        if phase == 'in':
            # The mod enters from a charged hold. The donor enters from ready.
            vm.pending[-1]['runtime_fade_in'] = max(.2, vm.pending[-1]['runtime_fade_in'])
        if reversal and phase in ('in', 'out'):
            event = 2149 if phase == 'in' else 2150
            when = vm.sequence_event_time(model, clip, event, None)
            if when is None:
                raise ValueError(f'Missing sword entity event {event} in {model}:{clip}')
            item_events[phase] = offset + when
        if phase == 'idle':
            swing = vm.sequence_event_time(model, clip, 2107, None)
            if swing is not None:
                marks.append((offset + swing, 'swing'))
            hit = vm.sequence_event_time(model, clip, 2102, None)
            if hit is None or not 0. < hit < natural:
                raise ValueError(f'Ground finisher hit event is absent from {model}:{clip}')
            marks.append((offset + hit, 'hit'))
        start_pose = poses[-1]
        segments.append((offset, times, [process_pose(pose) for pose in poses]))
        marks.append((offset, start_key))
        offset += float(times[-1])
        marks.append((offset, stop_key))
    if reversal:
        segments = [(start, times, [reversal.apply(pose, start + time, item_events['in'], item_events['out'])
                                    for time, pose in zip(times, poses)]) for start, times, poses in segments]
    Viewmodel.write(vm, folder, GROUP, segments, marks)
    return vm.groups[folder.name][GROUP]


def calibrated_pose(pose, factors):
    result = dict(pose)
    for name, (left, right) in factors.items():
        rotation, translation = result[name]
        result[name] = left @ rotation @ right, translation
    return result


def sword_pose(vm, pose):
    pose = vm.native_pose(pose)
    if vm.shield:
        return pose
    world = vm.fk(pose)
    left, right = (world[f'Bip01 {side} Hand'][0, 3] for side in 'LR')
    # Retarget the open hand outside the grip. The native forearms are
    # thicker than the donor's, especially with armor equipped.
    clearance = np.clip(9. - (right - left), 0., 10.)
    clavicle = 'Bip01 L Clavicle'
    rotation, translation = pose[clavicle]
    parent = world[vm.parents[clavicle]][:3, :3]
    pose[clavicle] = rotation, translation - parent.T @ np.array([clearance, 0., 0.])
    return pose


def dagger_pose(vm, factors):
    palms = {}
    for side in 'LR':
        roots = np.array([vm.local(f'Bip01 {side} Finger{index}')[1] for index in range(1, 5)])
        palms[f'Bip01 {side} Hand'] = frame(roots.mean(0), roots[3] - roots[0])

    def process(pose):
        pose = calibrated_pose(pose, factors)
        for name, palm in palms.items():
            rotation, translation = pose[name]
            correction, _ = lateral_cap(rotation, translation, palm)
            pose[name] = rotation @ correction, translation
        return pose

    return process


def dagger_offhand(arms, folder):
    # The active short blade uses the sword strike. Keep the other knife
    # seated in the calibrated idle grip, on the left of the strike.
    idle = folder / 'idle1h.kf'
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    channels, marks = tracks(idle), native_starts(idle)
    ready = Pose(rig, channels, marks['idle1h: start'])
    held = Pose(rig, channels, marks['idle1h: dagger primitive left forward slow stop'])
    item = ready.local(FRAME_NODES['L']) @ held.local(LEFT_WEAPON)
    result = {}
    for name in channels:
        if name.startswith('Bip01 L '):
            transform = ready.local(name)
            result[name] = (transform[:3, :3], transform[:3, 3])
    result[LEFT_WEAPON] = (item[:3, :3], item[:3, 3])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vpk-root', 'work-dir', 'skeleton', 'grip-animation', 'camera-contract', 'arms', 'calibration'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    shared = (args.vpk_root, args.work_dir, args.skeleton, args.grip_animation)
    camera = dict(camera_contract=args.camera_contract, native_eye_height=True)
    animations = args.arms / 'Animations'
    declaration = json.loads((args.arms / 'viewmodel-import.json').read_text())
    shield = declaration['shield_geometry_profile']
    grip = json.loads((args.calibration / 'sword-grip.json').read_text())
    _, arm = load_calibration(args.calibration / 'arm-frame.json', args.skeleton)
    report = {}

    sword = NativeSword(*shared, **camera, grip_profile=grip, shield_normal=declaration['shield_normal'])
    reversal = SwordReversal(sword)
    sword.shield_offset, sword.shield_scale = np.asarray(shield['offset_item_axes']), shield['scale']
    for suffix, folder in (('noshield', 'brutal_fp'), ('shield', 'brutal_fp_shield')):
        sword.shield = suffix == 'shield'
        model = 'first_animations_shield' if sword.shield else 'first_animations_sword'
        report[folder] = build(sword, animations / folder, model, 'sword_' + suffix,
                               lambda pose: sword_pose(sword, pose), reversal)

    staff = StaffNativeGrasp(*shared, **camera, grasp_profile=args.calibration / 'staff-native-grasp.json')
    staff.grip_path = args.grip_animation
    staff.configure('staff')
    staff_frames = corrections({'R': arm['R']})
    folder = staff.config['folder']

    report[folder] = build(staff, animations / folder, staff.config['model'], 'staff',
                           lambda pose: calibrated_pose(pose, staff_frames))

    sword.shield = False
    folder = 'brutal_fp_dagger'
    offhand = dagger_offhand(args.arms, animations / folder)
    report[folder] = build(sword, animations / folder, 'first_animations_sword', 'sword_noshield',
                           lambda pose: sword.native_pose(pose) | offhand, reversal)
    (args.arms / 'ground-finisher-import.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
