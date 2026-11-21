#!/usr/bin/env python3
"""Retarget Dark Messiah's dual-dagger stealth backstab animations into brutal_fp_dagger."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from calibrate_arm_frames import corrections, load_calibration
from import_dagger_anatomy import DaggerAnatomy, FRAME_NODES
from import_ground_finisher import dagger_pose
from import_viewmodel import Viewmodel
from retarget_kf import quat_matrix

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/io_scene_mw/lib'))
from es3 import nif


MODEL = 'first_animations_daggers'
GROUPS = ('brutal_backstab_in', 'brutal_backstab_idle', 'brutal_backstab_cancel', 'brutal_backstab_out')


def event(vm, clip, code, duration):
    time = vm.sequence_event_time(MODEL, clip, code, None)
    if time is None or not 0. < time < duration:
        raise ValueError(f'Backstab event {code} missing from {MODEL}:{clip}')
    return float(time)


def apply_grip_mount(path, info, profiles):
    """Apply calibration/dagger-grip.json identically to dagger_anatomy_grip_mount.py."""
    scene = nif.NiStream()
    scene.load(path)
    names = [e.string_data for e in scene.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    controllers = list(scene.root.controllers)
    keys = next(e for e in scene.root.extra_datas if isinstance(e, nif.NiTextKeyExtraData)).keys
    stop = next(float(time) for time, key in keys if key == path.stem + ': stop')
    for side, profile in profiles.items():
        anchor = np.asarray(profile['anchor_native_hand'])
        point = np.asarray(profile['hilt_point'])
        frame_name = FRAME_NODES[side]
        item_name = 'Weapon Bone' if side == 'R' else 'Brutal Left Weapon'
        frame_ctrl = controllers[names.index(frame_name)].data
        item_ctrl = controllers[names.index(item_name)].data
        frame_ctrl.translations.keys = np.asarray(
            [np.r_[row[0], anchor] for row in frame_ctrl.translations.keys], np.float32)
        translations = []
        for row in item_ctrl.rotations.keys:
            time = float(row[0])
            rotation = quat_matrix(row[1:][[1, 2, 3, 0]])
            if time <= stop + 1e-5:
                value = anchor - rotation @ point
            else:
                primitive = next((p for p in info['dagger_item_primitives'] if p['side'] == side
                                  and p['start'] - 1e-5 <= time <= p['stop'] + 1e-5), None)
                if primitive is None:
                    raise ValueError(f'Rotation outside playable/primitive ranges: {path}')
                value = -rotation @ point
            translations.append(np.r_[time, value])
        item_ctrl.translations.keys = np.asarray(translations, np.float32)
    scene.save(path)


def build_all(vm, folder, process_pose):
    ready, ready_fade = vm.stance_pose()

    # 1. Enter backstab stance (raises daggers behind shoulder / poised offhand).
    roll_in = {}
    dur_in = vm.duration(MODEL, 'anm_backstab_in')
    t_in, p_in, _ = vm.segment(MODEL, 'anm_backstab_in', duration=dur_in, roll_state=roll_in,
                               start_pose=ready, return_to=None, runtime_entry=True)
    raised = p_in[-1]
    Viewmodel.write(vm, folder, 'brutal_backstab_in',
                    [(0., t_in, [process_pose(p) for p in p_in])],
                    [(0., 'start'), (event(vm, 'anm_backstab_in', 2177, dur_in), 'hide offhand'),
                     (float(t_in[-1]), 'stop')])

    # 2. Looping backstab-ready hold.
    roll_idle = dict(roll_in)
    dur_idle = vm.duration(MODEL, 'anm_backstab_idle')
    t_idle, p_idle, _ = vm.segment(MODEL, 'anm_backstab_idle', duration=dur_idle, roll_state=roll_idle,
                                   start_pose=raised, return_to=None, runtime_entry=False)
    Viewmodel.write(vm, folder, 'brutal_backstab_idle',
                    [(0., t_idle, [process_pose(p) for p in p_idle])],
                    [(0., 'start'), (float(t_idle[-1]), 'stop')])

    # 3. Cancel stance back to ready idle.
    roll_cancel = dict(roll_in)
    dur_cancel = vm.duration(MODEL, 'anm_backstab_cancel')
    t_cancel, p_cancel, _ = vm.segment(MODEL, 'anm_backstab_cancel', duration=dur_cancel, roll_state=roll_cancel,
                                       start_pose=raised, return_to=(ready, ready_fade), runtime_entry=False)
    Viewmodel.write(vm, folder, 'brutal_backstab_cancel',
                    [(0., t_cancel, [process_pose(p) for p in p_cancel])],
                    [(0., 'start'), (event(vm, 'anm_backstab_cancel', 2178, dur_cancel), 'show offhand'),
                     (float(t_cancel[-1]), 'stop')])

    # 4. Execute backstab (twin stabs + neck slash + ragdoll release).
    roll_out = dict(roll_in)
    dur_out = vm.duration(MODEL, 'anm_backstab_out')
    head_out = vm.duration('player_head_mvt', '@layer_backstab_out')
    settle = (ready, max(ready_fade, head_out - dur_out))
    t_out, p_out, _ = vm.segment(MODEL, 'anm_backstab_out', duration=dur_out, roll_state=roll_out,
                                 start_pose=raised, return_to=settle, runtime_entry=False)
    marks_out = [
        (0., 'start'),
        (event(vm, 'anm_backstab_out', 2171, dur_out), 'align'),
        (event(vm, 'anm_backstab_out', 2102, dur_out), 'stab1'),
        (event(vm, 'anm_backstab_out', 2139, dur_out), 'stab2'),
        (event(vm, 'anm_backstab_out', 2166, dur_out), 'release'),
        (event(vm, 'anm_backstab_out', 2178, dur_out), 'show offhand'),
        (float(t_out[-1]), 'stop'),
    ]
    Viewmodel.write(vm, folder, 'brutal_backstab_out',
                    [(0., t_out, [process_pose(p) for p in p_out])],
                    marks_out)
    return {group: vm.groups[folder.name][group] for group in GROUPS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vpk-root', 'work-dir', 'skeleton', 'grip-animation', 'camera-contract', 'arms', 'calibration'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    shared = (args.vpk_root, args.work_dir, args.skeleton, args.grip_animation)
    camera = dict(camera_contract=args.camera_contract, native_eye_height=True)
    _, arm = load_calibration(args.calibration / 'arm-frame.json', args.skeleton)
    grip_profiles = json.loads((args.calibration / 'dagger-grip.json').read_text())

    dagger = DaggerAnatomy(*shared, **camera)
    dagger.grip_path = args.grip_animation
    dagger.configure('dagger')
    folder = args.arms / 'Animations' / dagger.config['folder']
    report = build_all(dagger, folder, dagger_pose(dagger, corrections(arm)))
    dagger.annotate_items(args.arms, {'dagger': {'groups': dagger.groups}})
    for group in GROUPS:
        apply_grip_mount(folder / (group + '.kf'), dagger.groups['brutal_fp_dagger'][group], grip_profiles)
    (args.arms / 'backstab-import.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
