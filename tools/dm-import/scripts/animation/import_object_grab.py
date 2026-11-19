#!/usr/bin/env python3
"""Retarget Dark Messiah's first-person object grab, hold, drop and throw animations."""
import argparse
import json
from pathlib import Path

import numpy as np

from import_sword_anatomy import SwordAnatomy
from import_viewmodel import BASIS, FPS, Viewmodel, to4
from retarget_anatomical_arm import follow_finger_directions
from retarget_kf import matrix_quat, quat_matrix


def eval_delta_world(buf, bones, base_tracks, delta_tracks=None):
    """Evaluate a Dark Messiah world pose from a base clip plus optional STUDIO_DELTA pose."""
    world = {}
    for bone in bones:
        t = base_tracks.get(bone.name)
        pos = np.array(t['pos'][0], float) if t else np.array(bone.position, float)
        rot = np.array(t['rot'][0], float) if t else np.array(bone.quat, float)
        mat = quat_matrix(rot)
        if delta_tracks and bone.name in delta_tracks:
            dt = delta_tracks[bone.name]
            pos = pos + np.array(dt['pos'][0], float)
            mat = mat @ quat_matrix(np.array(dt['rot'][0], float))
        local = to4(mat, pos)
        world[bone.name] = local if bone.parent_id < 0 else world[bones[bone.parent_id].name] @ local
    return world


def blend_poses(first, second, weight):
    """Smoothly blend two complete native pose dictionaries."""
    return Viewmodel.blend_pose(first, second, float(weight))


def smoothstep(u):
    u = float(np.clip(u, 0., 1.))
    return u * u * (3. - 2. * u)


class ObjectGrabImporter(SwordAnatomy):
    def __init__(self, *args, **kwargs):
        super().__init__(
            *args,
            anatomical_hand_target=True,
            reference_hand_frame=True,
            source_left_fingers=True,
            left_mcp_anchor='actual',
            **kwargs,
        )

    def pose_from_dm(self, dm_world, roll_state=None):
        pose = self.solve(dm_world, roll_state if roll_state is not None else {}, weapon_grip=False)
        local = {name: self.local(name) for name in self.nodes}
        for side in ('L', 'R'):
            pose = follow_finger_directions(pose, dm_world, self.fk(pose), local, self.parents, BASIS, side)
        return pose


def sample_segment(times, pose_fn):
    return [pose_fn(float(t)) for t in times]


def build_grab_group(vm, folder, group, neutral_pose, hold_pose, hold_alt_pose, raise_target_pose,
                     pickup_dur, hold_dur, raise_dur, drop_dur, raise_head_clip):
    """Build brutal_grab_{solo,dual}.kf with pickup, loopable hold, raise and drop sections."""
    t_pickup = np.linspace(0., pickup_dur, max(2, round(pickup_dur * FPS) + 1))
    p_pickup = sample_segment(t_pickup, lambda t: blend_poses(neutral_pose, hold_pose, smoothstep(t / pickup_dur)))

    # Smooth breathing sway around hold_pose so loop wraps seamlessly at hold start/stop.
    t_hold = np.linspace(0., hold_dur, max(2, round(hold_dur * FPS) + 1))
    p_hold = sample_segment(
        t_hold,
        lambda t: blend_poses(hold_pose, hold_alt_pose, 0.5 * (1. - np.cos(2. * np.pi * t / hold_dur))),
    )

    t_raise = np.linspace(0., raise_dur, max(2, round(raise_dur * FPS) + 1))
    p_raise = sample_segment(t_raise, lambda t: blend_poses(hold_pose, raise_target_pose, smoothstep(t / raise_dur)))

    t_drop = np.linspace(0., drop_dur, max(2, round(drop_dur * FPS) + 1))
    p_drop = sample_segment(t_drop, lambda t: blend_poses(hold_pose, neutral_pose, smoothstep(t / drop_dur)))

    offset = 0.
    segments = []
    marks = []
    vm.pending = []
    sections = [
        ('pickup', t_pickup, p_pickup, False, None),
        ('hold', t_hold, p_hold, True, None),
        ('raise', t_raise, p_raise, False, raise_head_clip),
        ('drop', t_drop, p_drop, False, None),
    ]
    for name, times, poses, is_loop, head_clip in sections:
        dur = float(times[-1])
        vm.pending.append(dict(
            model='first_animations_basic',
            clip=head_clip or 'anm_grab_' + ('dual' if 'dual' in group else 'solo'),
            natural=dur,
            loop=is_loop,
            fade_in=0.,
            runtime_fade_in=0.20 if name == 'pickup' else 0.,
            loop_period=dur if is_loop else None,
            loop_lead_in=0.,
            body_camera=None,
            head_camera=None,
            return_fade=0.,
        ))
        segments.append((offset, times, poses))
        marks.append((offset, f'{name} start'))
        offset += dur
        marks.append((offset, f'{name} stop'))

    Viewmodel.write(vm, folder, group, segments, marks)
    # Override camera-segments entry for raise so import_camera_head maps it to our sliced head clip.
    cam_seg = folder / 'camera-segments' / f'{group}.json'
    cam_seg.write_text(json.dumps([
        dict(start='raise start', stop='raise stop', model='player_head_mvt',
             clip=raise_head_clip, natural=raise_dur, duration=raise_dur)
    ], indent=1) + '\n')
    return vm.groups[folder.name][group]


def build_throw_group(vm, folder, group, dm_clip, head_clip, recover_ratio):
    """Build brutal_throw_{solo,dual}.kf from DM @layer_throw_{solo,dual}_high with exact release marker."""
    natural = vm.duration('first_animations_basic', dm_clip)
    release = vm.sequence_event_time('first_animations_basic', dm_clip, 2118, natural * 0.15)
    times = np.linspace(0., natural, max(2, round(natural * FPS) + 1))
    roll_state = {}
    poses = []
    for t in times:
        dm = vm.dm_world('first_animations_basic', dm_clip, float(t))
        poses.append(vm.pose_from_dm(dm, roll_state))

    vm.pending = [dict(
        model='first_animations_basic',
        clip=dm_clip,
        natural=natural,
        loop=False,
        fade_in=0.,
        runtime_fade_in=0.05,
        loop_period=None,
        loop_lead_in=0.,
        body_camera=None,
        head_camera=None,
        return_fade=0.,
    )]
    marks = [
        (0., 'start'),
        (float(release), 'release'),
        (float(natural * recover_ratio), 'recover'),
        (float(natural), 'stop'),
    ]
    Viewmodel.write(vm, folder, group, [(0., times, poses)], marks)
    cam_seg = folder / 'camera-segments' / f'{group}.json'
    cam_seg.write_text(json.dumps([
        dict(start='start', stop='stop', model='player_head_mvt',
             clip=head_clip, natural=natural, duration=natural)
    ], indent=1) + '\n')
    return dict(vm.groups[folder.name][group], release_time=float(release), duration=float(natural))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vpk-root', 'work-dir', 'skeleton', 'grip-animation', 'camera-contract', 'arms', 'calibration'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()

    vm = ObjectGrabImporter(
        args.vpk_root, args.work_dir, args.skeleton, args.grip_animation,
        camera_contract=args.camera_contract, native_eye_height=True,
    )
    buf, _, bones, anims = vm.dm_model('first_animations_basic')
    by_name = {a.name: a for a in anims}
    neut = by_name['a_grab_neutral'].read_animations(buf, bones)
    solo_delta = by_name['a_grab_solo_min'].read_animations(buf, bones)
    dual_min_delta = by_name['a_grab_dual_min'].read_animations(buf, bones)
    dual_max_delta = by_name['a_grab_dual_max'].read_animations(buf, bones)

    dm_neutral = eval_delta_world(buf, bones, neut, None)
    dm_solo = eval_delta_world(buf, bones, neut, solo_delta)
    dm_dual_min = eval_delta_world(buf, bones, neut, dual_min_delta)
    dm_dual_max = eval_delta_world(buf, bones, neut, dual_max_delta)

    neutral_pose = vm.pose_from_dm(dm_neutral)
    solo_hold_pose = vm.pose_from_dm(dm_solo)
    dual_min_pose = vm.pose_from_dm(dm_dual_min)
    dual_max_pose = vm.pose_from_dm(dm_dual_max)
    dual_hold_pose = blend_poses(dual_min_pose, dual_max_pose, 0.5)

    # Subtle hold sway poses: blend slightly toward relaxed or wider span without shifting far.
    solo_sway_dm = eval_delta_world(buf, bones, neut, solo_delta)
    solo_sway_dm = {k: v.copy() for k, v in solo_sway_dm.items()}
    solo_alt_pose = blend_poses(solo_hold_pose, neutral_pose, 0.04)
    dual_alt_pose = blend_poses(dual_hold_pose, dual_max_pose, 0.08)

    solo_throw_0 = vm.pose_from_dm(vm.dm_world('first_animations_basic', '@layer_throw_solo_high', 0.))
    dual_throw_0 = vm.pose_from_dm(vm.dm_world('first_animations_basic', '@layer_throw_dual_high', 0.))

    folder = args.arms / 'Animations/brutal_fp'
    report = {}
    report['brutal_grab_solo'] = build_grab_group(
        vm, folder, 'brutal_grab_solo', neutral_pose, solo_hold_pose, solo_alt_pose, solo_throw_0,
        pickup_dur=0.25, hold_dur=1.20, raise_dur=0.20, drop_dur=0.22,
        raise_head_clip='@layer_throw_solo_raise',
    )
    report['brutal_grab_dual'] = build_grab_group(
        vm, folder, 'brutal_grab_dual', neutral_pose, dual_hold_pose, dual_alt_pose, dual_throw_0,
        pickup_dur=0.30, hold_dur=1.40, raise_dur=0.24, drop_dur=0.25,
        raise_head_clip='@layer_throw_dual_raise',
    )
    report['brutal_throw_solo'] = build_throw_group(
        vm, folder, 'brutal_throw_solo', '@layer_throw_solo_high', '@layer_throw_solo_throw', recover_ratio=0.72,
    )
    report['brutal_throw_dual'] = build_throw_group(
        vm, folder, 'brutal_throw_dual', '@layer_throw_dual_high', '@layer_throw_dual_throw', recover_ratio=0.72,
    )

    # Record outputs in the manifest JSON for verification/audits.
    (args.arms / 'object-grab-import.json').write_text(json.dumps({
        'converter': 'import_object_grab.py',
        'derived_from_game_files': True,
        'groups': {k: {key: val for key, val in v.items() if key != 'segments'} for k, v in report.items()},
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
