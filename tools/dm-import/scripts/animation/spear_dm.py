#!/usr/bin/env python3
"""Spear set made of Dark Messiah's own staff thrusts.

The spear keeps the staff's guard, transitions and swings. Its quick strokes
A, B, C, D become DM's quick staff thrusts (B, D, B, D) and its charged attack
stays DM's charged staff jab (one-handed thrust, then two-handed thrust). The
authored arm, finger and regrip motion is copied unchanged; only the labels
that name the strokes change, plus phase keys (plant, weight forward, settle
end) that the head-motion and footwork layers read.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

from import_two_hand_viewmodel import keys, load, native_starts
from audit_staff_mesh_grip import nif

# Spear stroke -> DM source. Like the staff, the series opens with a quick
# stroke: A is the one-handed first jab of DM's charged staff thrust (0.26 s
# from start to contact). B, C, D are DM's quick staff thrusts with their
# wide regrip windups (0.85-0.91 s, as on the staff).
STROKES = {'a': 'jab', 'b': 'b', 'c': 'd', 'd': 'b'}
JAB_BEFORE, JAB_AFTER = .26, .23
GAP = .2


def build(arms, body=None):
    folder = arms / 'Animations/brutal_fp_staff'
    source = folder / 'weapononehand.kf'
    target = folder.with_name('brutal_fp_spear')
    target.mkdir(exist_ok=True)
    stream = load(source)
    text = keys(stream)
    marks = {line.removeprefix('weapononehand: '): float(t)
             for t, label in text.keys for line in label.splitlines()}
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    controllers = dict(zip(names, stream.root.controllers))

    cursor = max(float(t) for t, _ in text.keys) + 1.
    copies, labels = [], []

    def copy(src_start, src_stop, name, events):
        nonlocal cursor
        offset = cursor - src_start
        copies.append((src_start, src_stop, offset))
        labels.extend((t + offset, 'weapononehand: ' + name + ' ' + e) for t, e in events)
        cursor += (src_stop - src_start) + GAP

    for spear, dm in STROKES.items():
        if dm == 'jab':
            hit = marks['thrust hit']
            s0, s1 = hit - JAB_BEFORE, hit + JAB_AFTER
            sw = hit - .12
            end_source = 'a'
        else:
            s0, sw, hit, s1 = (marks[f'fast{dm} ' + k] for k in ('start', 'swing', 'thrust hit', 'stop'))
            end_source = dm
        events = [(s0, 'start'), (sw - .04, 'plant'), (sw, 'swing'), (hit, 'thrust hit'),
                  (hit + .035, 'weight forward'), (min(hit + .07, s1), 'settle end'), (s1, 'stop')]
        for name in (('fasta', 'fastloopa') if spear == 'a' else ('fast' + spear,)):
            copy(s0, s1, name, events)
        if f'fastend{end_source} start' in marks:
            e0, e1 = marks[f'fastend{end_source} start'], marks[f'fastend{end_source} stop']
        else:
            # DM's stroke D already ends in guard: its return is its last frames.
            e0, e1 = s1 - .05, s1
        copy(e0, e1, 'fastend' + spear, [(e0, 'start'), (e1, 'stop')])
    # Phase keys of the charged jab for the head motion and footwork.
    labels += [(marks['thrust min hit'], 'weapononehand: thrust plant'),
               (marks['thrust hit'] + .035, 'weapononehand: thrust weight forward'),
               (marks['thrust hit'] + .07, 'weapononehand: thrust settle end')]

    # Append the copied key ranges for every controller.
    for name, controller in controllers.items():
        for field in (controller.data.rotations, controller.data.translations):
            old = field.keys
            if old is None or not len(old):
                continue
            extra = []
            for start, stop, offset in copies:
                inside = old[(old[:, 0] >= start - 1e-5) & (old[:, 0] <= stop + 1e-5)].copy()
                inside[:, 0] += offset
                extra.append(inside)
            merged = np.vstack([old] + extra).astype(np.float32)
            merged = merged[np.argsort(merged[:, 0], kind='stable')]
            field.keys = merged
        controller.stop_time = max(controller.stop_time, cursor)

    replaced = ('fasta ', 'fastb ', 'fastc ', 'fastd ', 'fastend', 'fastloopa ')
    kept = []
    for t, label in text.keys:
        lines = [l for l in label.splitlines() if not l.removeprefix('weapononehand: ').startswith(replaced)]
        if lines:
            kept.append((float(t), '\r\n'.join(lines)))
    kept += labels + [(0., 'weapononehand: brutal spear grip')]
    kept.sort(key=lambda p: p[0])
    text.keys = np.array(kept, dtype=text.keys.dtype)
    stream.save(target / source.name)
    shutil.copyfile(source.with_suffix('.yaml'), target / 'weapononehand.yaml')
    for path in sorted(folder.glob('*1h.kf')):
        if path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            shutil.copyfile(path, target / path.name)
            if path.with_suffix('.yaml').exists():
                shutil.copyfile(path.with_suffix('.yaml'), target / path.with_suffix('.yaml').name)
    if body is not None:
        from import_spear_thrust import body_footwork
        body_footwork(body, target / source.name)
    manifest_path = arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())
    for path in sorted(target.iterdir()):
        manifest['outputs'][path.relative_to(arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['spear'] = {'base': 'brutal_fp_staff', 'quick': {k: ('dm charged first jab' if v == 'jab' else 'dm fast' + v) for k, v in STROKES.items()},
                         'charged': 'dm staff charged jab'}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arms', type=Path, required=True)
    parser.add_argument('--body', type=Path)
    args = parser.parse_args()
    build(args.arms, args.body)
