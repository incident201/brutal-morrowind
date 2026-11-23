#!/usr/bin/env python3
"""Retarget Dark Messiah's shieldless sword cleaning clip to the player arms."""

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from arm_steering import LocalPose, ease, steer_arm
from import_native_sword_visual import NativeSword
from import_staff_native_grasp import swing
from import_two_hand_viewmodel import ARM, bone, inverse
from import_viewmodel import Viewmodel, to4


GROUP = 'brutal_sword_wipe'
ALIGN, CONTACT_START, CONTACT_END, RELEASE = 4 / 30, 12 / 30, 45 / 30, 55 / 30
# The runtime fits the same path to the held model's blade in npcblood.cpp.
REFERENCE_LENGTH = 60.
BLADE_START = .135


def fit_wipe(vm, times, poses):
    # Use the source's closed gesture at the guard as the hand-to-blade grip.
    grip = poses[15]
    reference = LocalPose(vm, {name: to4(*value) for name, value in grip.items()})
    ref_weapon = reference.world('Weapon Bone')
    wrist_offsets = {side: inverse(ref_weapon) @ reference.world(bone(side, 'Hand')) for side in 'RL'}
    # On the reference iron blade, this section runs through the curled fingers.
    wrist_offsets['L'][1, 3] -= 8.6
    arms = {}
    for side in 'RL':
        names = [bone(side, part) for part in ARM]
        arms[side] = SimpleNamespace(names=names,
            segments=[np.linalg.norm(vm.local(name)[1]) for name in names[2:]])

    for time, source in zip(times, poses):
        weight = ease((time - ALIGN) / (CONTACT_START - ALIGN)) * ease((RELEASE - time) / (RELEASE - CONTACT_END))
        if weight <= 0.:
            yield source
            continue
        progress = ease((time - CONTACT_START) / (CONTACT_END - CONTACT_START))
        eye = vm.fk(source)['Camera'][:3, 3]
        # Present the flat across the view, then give the clearing hand room at the tip.
        axis = np.array([-1., .08 + .09 * progress, .035 - .055 * progress])
        weapon = to4(swing(ref_weapon[:3, 1], axis) @ ref_weapon[:3, :3],
            eye + np.array([21. + 6. * progress, 45. + 2. * progress, -14. + 2. * progress]))
        fitted = dict(source)
        fitted['Weapon Bone'] = grip['Weapon Bone']
        for side, arm in arms.items():
            wrist = wrist_offsets[side].copy()
            if side == 'L':
                wrist[1, 3] += REFERENCE_LENGTH * (BLADE_START + (1. - BLADE_START) * progress)
            joints = steer_arm(reference, reference, arm, weapon @ wrist, 0.)
            fitted.update((name, (matrix[:3, :3], matrix[:3, 3])) for name, matrix in joints.items())
        for name in fitted:
            if name.startswith('Bip01 L Finger'):
                fitted[name] = grip[name]
        yield vm.blend_pose(source, fitted, weight)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vpk-root', 'work-dir', 'skeleton', 'grip-animation', 'camera-contract', 'arms', 'calibration'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    grip = json.loads((args.calibration / 'sword-grip.json').read_text())
    manifest_path = args.arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())

    vm = NativeSword(args.vpk_root, args.work_dir, args.skeleton, args.grip_animation,
        grip_profile=grip, camera_contract=args.camera_contract, native_eye_height=True)
    times, poses, _ = vm.segment('first_animations_sword', 'anm_sword_noshield_cleanweapon', runtime_entry=True)
    end = float(times[-1])
    if end <= RELEASE:
        raise ValueError('The blade contact section lies outside the cleaning clip')
    poses = list(fit_wipe(vm, times, [vm.native_pose(pose) for pose in poses]))
    out = args.arms / 'Animations' / 'brutal_fp'
    # NativeSword.write collects sections for a different export. Use the standalone KF writer here.
    Viewmodel.write(vm, out, GROUP, [(0, times, poses)], [(0, 'start'), (ALIGN, 'align'),
        (CONTACT_START, 'contact start'), (CONTACT_END, 'contact end'), (RELEASE, 'release'), (end, 'stop')])
    for suffix in ('.kf', '.yaml'):
        path = out / (GROUP + suffix)
        manifest['outputs'][path.relative_to(args.arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    main()
