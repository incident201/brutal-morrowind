#!/usr/bin/env python3
"""Retarget Dark Messiah's first-person bow draw, hold, release and recovery.

Writes Animations/brutal_fp_bow/bowandarrow.kf and idlebow.kf. The bow group keeps
Morrowind's phase keys (equip, shoot start, attach, min/max attack, release,
follow, unequip) on Dark Messiah's timing; the engine maps the bow mesh's own
bend and arrow-bone animation onto these phases.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from import_object_grab import ObjectGrabImporter, blend_poses, smoothstep
from import_viewmodel import FPS, Viewmodel

MODEL = 'first_animations_bow'
# The bow outside the attack: ready idle, draw (equip), undraw (unequip), walking and running.
BASIC = 'first_animations_basic'
BOW_MODEL = 'items/weapons/bow_classic/bow_classic'
LEFT_MOUNT = 'Brutal Left Weapon'
# Morrowind bows: length along +Y, string on -X (the arrow flies towards +X), flat across Z.
# Dark Messiah's bow after its @idle, measured in the hand at full draw: length along +Z, the arrow flies
# towards -Y (the string is on +Y), the limbs' width along X.
# Columns: the Dark Messiah item axes of Morrowind's X, Y and Z.
MW_IN_DM_BOW = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
# Middle of the grip along a Morrowind bow's length (vanilla w_longbow spans -38.7..45.9).
MW_GRIP_CENTER = np.array([0., 3.6, 0.])


def source(clip, natural, loop=False, runtime_fade_in=0., model=MODEL):
    return dict(model=model, clip=clip, natural=natural, loop=loop, fade_in=0., runtime_fade_in=runtime_fade_in,
                loop_period=natural if loop else None, loop_lead_in=0., body_camera=None, head_camera=None,
                return_fade=0.)


def frames(duration):
    return np.linspace(0., duration, max(2, round(duration * FPS) + 1))


def clip_poses(vm, clip, start, stop, speed=1., model=MODEL):
    """Poses of a Dark Messiah clip between start and stop seconds, played at speed."""
    times = frames((stop - start) / speed)
    roll = {}
    return times, [vm.pose_from_dm(vm.dm_world(model, clip, start + float(t) * speed), roll) for t in times]


def ease_in(first, times, poses, duration):
    """Blend from a pose into the start of a clip."""
    return [blend_poses(first, pose, smoothstep(float(t) / duration)) for t, pose in zip(times, poses)]


def hold(pose, duration):
    times = frames(duration)
    return times, [pose for _ in times]


def blend(first, second, duration):
    times = frames(duration)
    return times, [blend_poses(first, second, smoothstep(float(t) / duration)) for t in times]


def add_sound_keys(vm, path, sounds):
    """Add Morrowind 'Sound: <id>' text keys (they carry no group prefix)."""
    nif = vm.nif
    stream = nif.NiStream()
    stream.load(path)
    extra = next(e for e in stream.root.extra_datas if isinstance(e, nif.NiTextKeyExtraData))
    keys = [(float(t), str(k)) for t, k in extra.keys] + [(float(t), 'Sound: ' + sound) for t, sound in sounds]
    keys.sort(key=lambda key: key[0])
    extra.keys = np.array(keys, dtype='<f,O')
    stream.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vpk-root', 'work-dir', 'skeleton', 'grip-animation', 'camera-contract', 'arms', 'calibration'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--bow-roll', type=float, default=0.,
                        help='Extra turn of the bow about its length, in degrees')
    parser.add_argument('--draw-speed', type=float, default=1.35,
                        help='Playback rate of the Dark Messiah draw (its first draw raises the bow at leisure)')
    args = parser.parse_args()

    vm = ObjectGrabImporter(args.vpk_root, args.work_dir, args.skeleton, args.grip_animation,
                            camera_contract=args.camera_contract, native_eye_height=True)

    # The bow sits rigidly in Dark Messiah's left hand (LeftHand_Bow). The retarget keeps the hands' DM
    # orientation (hand_map), so the Morrowind bow's mount is one fixed transform in the left hand.
    bow_attachment = vm.attachment('first_basic', 'LeftHand_Bow')
    bow_idle = vm.item_idle(BOW_MODEL)
    bow_in_hand = bow_attachment @ bow_idle
    hand_map_left, hand_map_right = vm.hand_map['L'], vm.hand_map['R']
    # The nocked arrow: its nock at RightHand_Arrows, pointing at the bow's rest (Arrows_Aim).
    arrow_nock = hand_map_right.T @ (vm.scale * vm.attachment('first_basic', 'RightHand_Arrows')[:3, 3])

    def mount_for(roll):
        turn = np.array([[np.cos(roll), 0., np.sin(roll)], [0., 1., 0.], [-np.sin(roll), 0., np.cos(roll)]])
        rotation = hand_map_left.T @ bow_in_hand[:3, :3] @ MW_IN_DM_BOW @ turn
        return rotation, hand_map_left.T @ (vm.scale * bow_in_hand[:3, 3]) - rotation @ MW_GRIP_CENTER

    if LEFT_MOUNT not in vm.nodes:
        vm.nodes[LEFT_MOUNT] = vm.nif.NiNode(name=LEFT_MOUNT)
        vm.parents[LEFT_MOUNT] = 'Bip01 L Hand'
    vm.keyed = list(dict.fromkeys(vm.keyed + [LEFT_MOUNT]))
    # Turn the bow about its length so its string plane meets the nock at full draw: Morrowind's string
    # morph pulls straight back in that plane, and the arrow sits on the string there.
    full_draw = vm.pose_from_dm(vm.dm_world(MODEL, 'anm_bowattack_idle', 0.))
    full_draw[LEFT_MOUNT] = mount_for(0.)
    world = vm.fk(full_draw)
    nock_world = world['Bip01 R Hand'][:3, :3] @ arrow_nock + world['Bip01 R Hand'][:3, 3]
    in_bow = world[LEFT_MOUNT][:3, :3].T @ (nock_world - world[LEFT_MOUNT][:3, 3])
    roll = np.arctan2(in_bow[2], -in_bow[0]) + np.radians(args.bow_roll)
    mount_rotation, mount_translation = mount = mount_for(roll)
    arrow_rest = hand_map_left.T @ (vm.scale * vm.attachment('first_basic', 'Arrows_Aim')[:3, 3])
    # Before it is nocked the arrow lies along the fingers: the arrow model's +X at RightHand_Arrows.
    arrow_hold = hand_map_right.T @ vm.attachment('first_basic', 'RightHand_Arrows')[:3, 0]
    pose_from_dm = vm.pose_from_dm

    def mounted(dm, roll_state=None):
        pose = pose_from_dm(dm, roll_state)
        pose[LEFT_MOUNT] = mount
        return pose
    vm.pose_from_dm = mounted
    draw_clip = 'anm_bowattack_in'
    draw_length = vm.duration(MODEL, draw_clip)
    # Dark Messiah events on the draw: 2112 takes the arrow, 2113 starts the pull, 2144 is fully drawn.
    take = vm.sequence_event_time(MODEL, draw_clip, 2112, draw_length * .25)
    pull = vm.sequence_event_time(MODEL, draw_clip, 2113, draw_length * .5)
    full = vm.sequence_event_time(MODEL, draw_clip, 2144, draw_length * .72)
    shoot_clip = 'anm_bowattack_shoot'
    shoot_length = vm.duration(MODEL, shoot_clip)
    release = vm.sequence_event_time(MODEL, shoot_clip, 2114, shoot_length * .14)

    ready = vm.pose_from_dm(vm.dm_world(BASIC, 'anm_bow_idle', 0.))
    drawn = vm.pose_from_dm(vm.dm_world(MODEL, 'anm_bowattack_idle', 0.))
    shot_end = vm.pose_from_dm(vm.dm_world(MODEL, shoot_clip, shoot_length))

    speed = args.draw_speed
    segments, marks, offset = [], [], 0.
    vm.pending = []

    def add(times, poses, src):
        nonlocal offset
        segments.append((offset, times, poses))
        vm.pending.append(src)
        offset += float(times[-1])

    def mark(label, at=None):
        marks.append((offset if at is None else at, label))

    # Equip: Dark Messiah's bow draw brings it into the ready pose.
    equip_length = vm.duration(BASIC, 'anm_bow_draw')
    mark('equip start')
    mark('equip attach', offset + equip_length * .4)
    times, poses = clip_poses(vm, 'anm_bow_draw', 0., equip_length, model=BASIC)
    add(times, poses, source('anm_bow_draw', equip_length, runtime_fade_in=.15, model=BASIC))
    add(*blend(poses[-1], ready, .15), source('anm_bow_idle', .15, model=BASIC))
    mark('equip stop')

    def add_shot(phase, clip, duration, take_at, pull_at, full_at, nock_at=None):
        start = offset
        mark(phase + ' start')
        times, poses = clip_poses(vm, clip, 0., full_at, speed)
        if nock_at is None:
            poses = ease_in(ready, times, poses, .2)
        add(times, poses, source(clip, duration, runtime_fade_in=.1))
        mark(phase + ' attach', start + take_at / speed)
        mark(phase + ' min attack', start + pull_at / speed)
        if nock_at is not None:
            mark(phase + ' nock', start + nock_at / speed)
        # Both draws reach the same hold and fire the same release.
        settle = .12 if nock_at is None else .07
        add(*blend(poses[-1], drawn, settle), source('anm_bowattack_idle', settle))
        mark(phase + ' max attack')
        add(*hold(drawn, 1. / FPS), source('anm_bowattack_idle', 1. / FPS))
        mark(phase + ' min hit')
        shot_start = offset
        times, poses = clip_poses(vm, shoot_clip, 0., shoot_length)
        add(times, poses, source(shoot_clip, shoot_length))
        mark(phase + ' release', shot_start + release)
        mark(phase + ' follow start')
        mark(phase + ' chain', offset + .12)
        add(*blend(shot_end, ready, .42), source(shoot_clip, .42))
        mark(phase + ' follow stop')

    add_shot('shoot', draw_clip, draw_length, take, pull, full)

    # Unequip: Dark Messiah's undraw.
    unequip_length = vm.duration(BASIC, 'anm_bow_undraw')
    mark('unequip start')
    mark('unequip detach', offset + unequip_length * .6)
    times, poses = clip_poses(vm, 'anm_bow_undraw', 0., unequip_length, model=BASIC)
    poses = ease_in(ready, times, poses, .1)
    add(times, poses, source('anm_bow_undraw', unequip_length, model=BASIC))
    mark('unequip stop')

    fast_clip = 'anm_bowattack_fastreload'
    fast_length = vm.duration(MODEL, fast_clip)
    fast_take = vm.sequence_event_time(MODEL, fast_clip, 2112, .30)
    fast_pull = vm.sequence_event_time(MODEL, fast_clip, 2113, .57)
    fast_full = vm.sequence_event_time(MODEL, fast_clip, 2144, .93)
    fast_nock = vm.sequence_event_time(MODEL, fast_clip, 2115, .50)
    add_shot('shoot fast', fast_clip, fast_length, fast_take, fast_pull, fast_full, fast_nock)

    folder = args.arms / 'Animations/brutal_fp_bow'
    folder.mkdir(parents=True, exist_ok=True)
    marks.sort(key=lambda m: m[0])
    Viewmodel.write(vm, folder, 'bowandarrow', segments, marks)
    times = dict((label, t) for t, label in marks)
    add_sound_keys(vm, folder / 'bowandarrow.kf', [
        key
        for phase in ('shoot', 'shoot fast')
        for key in ((times[phase + ' start'] + .05, 'Brutal Bow Take Arrow'),
                    (times[phase + ' min attack'], 'Brutal Bow Draw'),
                    (times[phase + ' release'], 'BowShoot'),
                    (times[phase + ' release'], 'Brutal Bow Release'))
    ])
    report = {'bowandarrow': {k: v for k, v in vm.groups[folder.name]['bowandarrow'].items() if k != 'segments'}}

    # Idle, walking and running with the bow ready: Dark Messiah's own loops.
    def loop(group, clip, speed=1.):
        length = vm.duration(BASIC, clip)
        times, poses = clip_poses(vm, clip, 0., length, speed, model=BASIC)
        period = float(times[-1])
        vm.pending = [source(clip, length, loop=True, model=BASIC)]
        Viewmodel.write(vm, folder, group, [(0., times, poses)],
                        [(0., 'start'), (0., 'loop start'), (period, 'loop stop'), (period, 'stop')])
        report[group] = {k: v for k, v in vm.groups[folder.name][group].items() if k != 'segments'}

    loop('idlebow', 'anm_bow_idle')
    for direction in ('forward', 'back', 'left', 'right'):
        loop(f'walk{direction}bow', '@walk_bow')
        loop(f'run{direction}bow', '@run_bow')
        loop(f'sneak{direction}bow', '@walk_bow', .7)
    (folder / 'arrow-mount.txt').write_text(
        'nock {:.4f} {:.4f} {:.4f}\nrest {:.4f} {:.4f} {:.4f}\nhold {:.4f} {:.4f} {:.4f}\n'.format(
            *arrow_nock, *arrow_rest, *arrow_hold))
    (args.arms / 'bow-import.json').write_text(json.dumps({
        'bow_mount': {'rotation': mount_rotation.tolist(), 'translation': mount_translation.tolist(),
                      'roll_degrees': float(np.degrees(roll)), 'nock_draw_back': float(-in_bow[0])},
        'arrow_mount': {'nock_right_hand': arrow_nock.tolist(), 'rest_left_hand': arrow_rest.tolist()},
        'converter': 'import_bow.py',
        'derived_from_game_files': True,
        'draw_speed': speed,
        'events': {'take': take, 'pull': pull, 'full': full, 'release': release},
        'fast_events': {'take': fast_take, 'nock': fast_nock, 'pull': fast_pull, 'full': fast_full},
        'groups': report,
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
