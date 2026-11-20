#!/usr/bin/env python3
"""Dark Messiah's first-person camera motion (player_head_mvt), for the Lua camera script.

DM plays one head clip on a separate entity: the walk_*/run_* clip of the movement
direction, megasprint_* while sprinting, jump_out_* on landing (mm_headbobbing, on by
default), and over that the @layer_* clip named like each arm action layer. This step
reads camera-segments/<group>.json, written by the arm steps beside each set, maps every part
of a group to its head clip and writes scripts/brutal_assets/camera_head.lua:

    clips = {[name] = {fps, loop, frames = {{pitch, yaw, roll, x, y, z}, ...}}}
    sets = {[folder] = {[group] = {{start, stop, clip, duration, natural}, ...}}}

Angles are Lua camera extras in radians; x right, y forward, z up are native eye
offsets in actor units. Lua finds each part between its two text keys at run time,
so re-timed and extended groups still line up.
"""
import argparse
import json
import shutil
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import import_viewmodel as iv  # noqa: E402
from import_two_hand_viewmodel import native_starts  # noqa: E402
from spear_dm import SWEEPS  # noqa: E402

LOCOMOTION = ([f'@walk_{d}' for d in ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')]
              + [f'@run_{d}' for d in ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')]
              + ['@megasprint_sword', '@megasprint_staff', '@megasprint_daggers',
                 '@jump_out_idle', '@jump_out_walk', '@jump_out_run', '@jump_out_sprint'])
FAMILY = {'brutal_fp': ('sword', 'noshield'), 'brutal_fp_shield': ('sword', 'shield'),
          'brutal_fp_staff': ('staff', None), 'brutal_fp_dagger': ('daggers', None)}


def candidates(folder, clip):
    """Head clips that DM plays with this arm clip, most specific first."""
    family, kind = FAMILY[folder]
    name = clip.lstrip('@')
    if name.startswith('anm_'):
        name = name[4:]
    out = ['@layer_'+name]
    if name.startswith('layer_'):
        out.insert(0, '@'+name)
    # Kicks: anm_sword_noshield_kick_medium -> @layer_sword_kick_medium, shield -> @layer_shield_kick_*.
    for height in ('medium', 'low'):
        if name.endswith('kick_'+height):
            out.append(f'@layer_{"shield" if kind == "shield" else family}_kick_{height}')
    # Parry hits: anm_parry_sword_shield_A -> @layer_parry_sword_shield_A (already), staff has none.
    # Draw: anm_sword_noshield_quickdraw -> @layer_sword_noshield_draw; daggers likewise.
    if name.endswith('quickdraw'):
        out.append('@layer_'+name.replace('quickdraw', 'draw'))
    # Combo names put the shield suffix last: anm_combo_sword_rush_noshield.
    if kind and f'_{kind}' in name:
        out.append('@layer_'+name.replace(f'_{kind}', '')+f'_{kind}')
    # The falling strike's head clip has no _out suffix: @layer_combo_sword_fall_noshield.
    if name.endswith('_out'):
        out.append('@layer_'+name[:-4])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vpk-root', type=Path, required=True)
    ap.add_argument('--work-dir', type=Path, required=True)
    ap.add_argument('--arms', type=Path, required=True, help='Finished first-person data (Animations/*/camera-segments/*.json)')
    ap.add_argument('--camera-contract', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    vm = iv.Viewmodel.__new__(iv.Viewmodel)
    vm.vpk, vm.work, vm.models, vm.sequences, vm.provenance = a.vpk_root, a.work_dir, {}, {}, []
    scale = json.loads(a.camera_contract.read_text())['body_scale']
    animations = vm.dm_model('player_head_mvt')[3]
    # Arm and head clips differ in case (anm_parry_Daggers_A, @layer_parry_daggers_A).
    available = {anim.name.lower(): anim for anim in animations}
    clips, sets, report = {}, {}, {}

    def sample(name, time):
        position, angles = vm.head_motion(name, time)
        return np.r_[angles, scale * iv.BASIS @ np.asarray(position, float)]

    def bake(name, previous=None):
        if name.lower() in ('@layer_throw_solo_raise', '@layer_throw_solo_throw',
                            '@layer_throw_dual_raise', '@layer_throw_dual_throw'):
            if name in clips:
                return name
            is_dual = 'dual' in name
            is_raise = name.endswith('_raise')
            source_name = '@layer_throw_dual_high' if is_dual else '@layer_throw_solo_high'
            anim = available.get(source_name.lower())
            if anim is None:
                return False
            split_frame = 7  # Frame 7 (~0.29s) is the peak cock-back right before the forward whip.
            f_start = 0 if is_raise else split_frame
            f_end = split_frame if is_raise else (anim.frame_count - 1)
            frames = [sample(source_name, frame / anim.fps).tolist() for frame in range(f_start, f_end + 1)]
            clips[name] = dict(fps=float(anim.fps), loop=False, frames=frames)
            return name
        anim = available.get(name.lower())
        if anim is None:
            return False
        name = anim.name
        if name in clips:
            return name
        fade = vm.sequence_fade('player_head_mvt', name) if previous else 0.
        frames = []
        for frame in range(anim.frame_count):
            time = frame / anim.fps
            pose = sample(name, time)
            # A new ground-impale layer starts on the arm boundary while its
            # outgoing camera is still playing, including the 30 fps shield.
            if time < fade:
                outgoing = sample(previous['clip'], previous['duration'] + time)
                amount = time / fade
                pose = (1. - amount) * outgoing + amount * pose
            frames.append(pose.tolist())
        clips[name] = dict(fps=float(anim.fps), loop=bool(int(anim.flags) & 1), frames=frames)
        return name

    for name in LOCOMOTION:
        if not bake(name):
            raise SystemExit(f'player_head_mvt has no {name}')
    for path in sorted(a.arms.glob('Animations/*/camera-segments/*.json')):
        folder, group = path.parent.parent.name, path.stem
        if folder not in FAMILY:
            continue
        for entries in (json.loads(path.read_text()),):
            if group.startswith('brutal_clinch'):
                continue  # The contest has its own camera curve (clinch_head.lua).
            parts = []
            for entry in entries:
                ground_finisher = group == 'brutal_ground_finisher'
                previous = parts[-1] if ground_finisher and parts else None
                head = next((found for clip in candidates(folder, entry['clip'])
                             if (found := bake(clip, previous))), None)
                if ground_finisher and not head:
                    raise ValueError(f"Ground finisher camera layer missing for {entry['clip']}")
                report.setdefault(folder, []).append([group, entry['start'], entry['clip'], head])
                if head:
                    duration = entry['duration']
                    if head.startswith('@layer_throw_') and head in clips and clips[head]['frames']:
                        duration = (len(clips[head]['frames']) - 1) / clips[head]['fps']
                    parts.append(dict(start=entry['start'], stop=entry['stop'], clip=head,
                                      duration=duration, natural=entry['natural']))
            if parts:
                sets.setdefault(folder, {})[group] = parts

    # The thrusts have their own shoulder motion. Preserve staff head layers
    # for the actions still played from that set, including the ground finisher.
    staff = sets['brutal_fp_staff']
    staff_keys = native_starts(a.arms / 'Animations/brutal_fp_staff/weapononehand.kf')
    spear_parts = []
    for part in staff['weapononehand']:
        donor = part['start'].split(' ', 1)[0]
        if donor in SWEEPS:
            sweep, _ = SWEEPS[donor]
            start, swing, stop = (staff_keys[f'weapononehand: {donor} {key}'] for key in ('start', 'swing', 'stop'))
            source = clips[part['clip']]
            # The spear joins the real attack at "swing". Remove the old
            # staff windup from the camera clip on the same normalized clock.
            last = min(round(part['duration'] * source['fps']), len(source['frames']) - 1)
            first = round(last * (swing - start) / (stop - start))
            frames = source['frames'][first:last + 1]
            name = part['clip'] + '_spear_swing'
            duration = (len(frames) - 1) / source['fps']
            clips[name] = dict(source, frames=frames)
            spear_parts.append(dict(start=sweep + ' swing', stop=sweep + ' recovery', clip=name,
                                    duration=duration, natural=duration))
        elif not part['start'].startswith(('thrust ', 'fast')):
            spear_parts.append(part)
    sets['brutal_fp_spear'] = {**staff, 'weapononehand': spear_parts}

    def lua(value, indent=''):
        if isinstance(value, dict):
            inner = indent+'  '
            items = []
            for key, item in value.items():
                k = key if key.isidentifier() else '['+json.dumps(key)+']'
                items.append(f'{inner}{k} = {lua(item, inner)},')
            return '{\n'+'\n'.join(items)+'\n'+indent+'}'
        if isinstance(value, list):
            if value and all(isinstance(v, float) for v in value):
                return '{'+', '.join(f'{v:.6g}' for v in value)+'}'
            inner = indent+'  '
            return '{\n'+'\n'.join(inner+lua(v, inner)+',' for v in value)+'\n'+indent+'}'
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if isinstance(value, float):
            return f'{value:.9g}'
        return json.dumps(value)

    dest = a.output/'scripts/brutal_assets/camera_head.lua'
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text('-- Dark Messiah first-person camera motion, derived from the user\'s own game files.\n'
                    'return '+lua(dict(clips=clips, sets=sets))+'\n')
    (a.output/'camera-head-import.json').write_text(json.dumps(dict(
        converter='import_camera_head.py', derived_from_game_files=True, body_scale=scale,
        clips=sorted(clips), mapping=report), indent=1)+'\n')
    for path in a.arms.glob('Animations/*/camera-segments'):
        shutil.rmtree(path)
    mapped = sum(1 for rows in report.values() for row in rows if row[3])
    print(f'{len(clips)} head clips, {mapped} of {sum(len(r) for r in report.values())} arm parts mapped')


if __name__ == '__main__':
    main()
