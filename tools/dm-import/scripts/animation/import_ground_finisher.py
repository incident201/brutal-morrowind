#!/usr/bin/env python3
"""Retarget the three ground-impale layers from the user's own game files."""
import argparse
import json
from pathlib import Path

import numpy as np

from calibrate_arm_frames import corrections, load_calibration
from import_dagger_anatomy import DaggerAnatomy
from import_native_sword_visual import NativeSword
from import_staff_native_grasp import StaffNativeGrasp
from import_viewmodel import Viewmodel, frame
from native_wrist_profile import lateral_cap


GROUP = 'brutal_ground_finisher'
PHASES = (('in', 'start', 'entry stop'), ('idle', 'strike start', 'strike stop'),
          ('out', 'return start', 'stop'))


def build(vm, folder, model, kind, process_pose):
    ready, ready_fade = vm.stance_pose()
    start_pose = ready
    roll_state = {}
    offset, segments, marks = 0., [], []
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
    Viewmodel.write(vm, folder, GROUP, segments, marks)
    return vm.groups[folder.name][GROUP]


def calibrated_pose(pose, factors):
    result = dict(pose)
    for name, (left, right) in factors.items():
        rotation, translation = result[name]
        result[name] = left @ rotation @ right, translation
    return result


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
    sword.shield_offset, sword.shield_scale = np.asarray(shield['offset_item_axes']), shield['scale']
    for suffix, folder in (('noshield', 'brutal_fp'), ('shield', 'brutal_fp_shield')):
        sword.shield = suffix == 'shield'
        model = 'first_animations_shield' if sword.shield else 'first_animations_sword'
        report[folder] = build(sword, animations / folder, model, 'sword_' + suffix, sword.native_pose)

    staff = StaffNativeGrasp(*shared, **camera, grasp_profile=args.calibration / 'staff-native-grasp.json')
    staff.grip_path = args.grip_animation
    staff.configure('staff')
    staff_frames = corrections({'R': arm['R']})
    folder = staff.config['folder']
    report[folder] = build(staff, animations / folder, staff.config['model'], 'staff',
                           lambda pose: calibrated_pose(pose, staff_frames))

    dagger = DaggerAnatomy(*shared, **camera)
    dagger.grip_path = args.grip_animation
    dagger.configure('dagger')
    folder = dagger.config['folder']
    report[folder] = build(dagger, animations / folder, dagger.config['model'], 'daggers',
                           dagger_pose(dagger, corrections(arm)))
    dagger.annotate_items(args.arms, {'dagger': {'groups': dagger.groups}})
    (args.arms / 'ground-finisher-import.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
