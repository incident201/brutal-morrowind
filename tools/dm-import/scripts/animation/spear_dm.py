#!/usr/bin/env python3
"""Spear thrusts made of Dark Messiah's one straight staff jab.

Every DM staff stroke that is labelled a thrust starts with an overhead swing,
except one: the second jab of the charged staff attack. There the haft keeps
pointing forward, the rear fist draws it back about 29 units while the left
hand regrips, then drives it 50 units forward in 0.15 s to the crosshair and
holds. Each stroke starts in that drawn chamber, so a series reads as
thrust, draw back, thrust. This module builds the spear's quick strokes and its charged thrust
from that jab. The guard, the staff swings and every transition stay the
staff's. The returns to guard are short blends instead of DM's overhead
recovery. Arms, fingers and regrips inside the jab are the artists'.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

from import_two_hand_viewmodel import ITEM, Pose, Rig, keys, load, matrix_quat, tracks
from import_spear_thrust import blend, ease
from audit_staff_mesh_grip import nif

# The straight jab inside DM's charged staff attack, relative to its hit2 key.
JAB_CHAMBER, JAB_END = -.19, .18   # deepest draw (both hands on the haft), end of the steady hold
SETTLE = .12          # each stroke holds the chamber while the engine blends into it
RETURN = .38          # draw back from the held thrust to the guard
RATE = 60.
# The staff's own sections (side swing, draw, sheathe) start and end in the
# staff guard, haft across the view with the point up and left. Blending from
# the spear guard straight into it swings the haft through the camera in a
# few frames. DM's side swing passes through a point-forward pose twice: the
# early wind-up (SWING_JOIN) and the recovery after the second impact
# (SWING_LEAVE). The spear joins and leaves the swing there.
SWING_JOIN, SWING_LEAVE = .5, .46     # after 'slash start' / after 'slash hit2'
JOIN, LEAVE = .4, .4                  # blend lengths into and out of the swing
DIP = (0., 6., -16.)                  # the hands' detour under the eye during those blends
DRAW_EARLY = .1                       # leave DM's draw this long before the attach key
# A person never strikes the same spot twice. Each quick stroke turns both
# arms and the spear together about the neck by its own (yaw, pitch, roll) in
# degrees: z up, positive yaw to the left, positive pitch tip up, positive roll
# clockwise from behind. The turn grows during the drive and goes back during
# the draw, so every stroke still starts and ends in the same guard.
AIM = {'fasta': (5.5, -2.2, -8.), 'fastb': (-6., 1.8, 9.), 'fastc': (1.6, -5.2, 4.),
       'fastd': (-3.6, -3., -10.), 'fastloopa': (6.4, 1., 6.), 'thrust': (0., -.8, 0.),
       'neutral': (-1.5, -1.5, -3.)}


def build(arms, body=None):
    folder = arms / 'Animations/brutal_fp_staff'
    source = folder / 'weapononehand.kf'
    target = folder.with_name('brutal_fp_spear')
    target.mkdir(exist_ok=True)
    stream = load(source)
    text = keys(stream)
    marks = {line.removeprefix('weapononehand: '): float(t)
             for t, label in text.keys for line in label.splitlines()}
    channels = tracks(source)               # original data, sampled below
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    controllers = dict(zip(names, stream.root.controllers))
    animated = [n for n in names if n in channels]

    hit2 = marks['thrust hit2']
    chamber, jab1 = hit2 + JAB_CHAMBER, hit2 + JAB_END
    # The spear's guard is the jab's drawn chamber: haft forward, both hands
    # on it. Thrusts start and end there without sweeping across the view.

    def at(t):
        pose = Pose(rig, channels, t)
        return {n: pose.local(n) for n in animated}

    guard_local = at(chamber)

    segments = []          # (start, stop, fn(elapsed) -> local transforms)
    labels = []

    def section(start, length, fn, name, events):
        segments.append((start, start + length, fn))
        labels.extend((start + t, 'weapononehand: ' + name + ' ' + e) for t, e in events)

    def back_to_guard(end_pose):
        return lambda e: {n: blend(end_pose[n], guard_local[n], ease(e / RETURN)) for n in animated}

    end_local = at(jab1)
    drive = jab1 - chamber

    reach = hit2 - chamber      # the drive itself, without the static hold

    from import_spear_thrust import LocalPose, steer_arm
    from import_two_hand_viewmodel import Grasp, ITEM as WEAPON, inverse
    contact_pose, guard_pose = Pose(rig, channels, hit2), Pose(rig, channels, chamber)
    left = Grasp(contact_pose, 'L', False)
    grip_contact = inverse(contact_pose.world(WEAPON)) @ contact_pose.world(left.names[-1])
    grip_guard = inverse(guard_pose.world(WEAPON)) @ guard_pose.world(left.names[-1])

    def returning(e):
        # Draw the haft back along the same line: DM's own drive, reversed,
        # from the contact pose. The static hold is not replayed; its last
        # 0.6-unit settle is closed in the first 0.04 s. The left hand does
        # not replay DM's regrip backwards: it slides smoothly from its
        # contact pose to its guard pose, holding the haft.
        if e < .04:
            return at(jab1 - (jab1 - hit2) * e / .04)
        u = ease(min(1., (e - .04) / (RETURN - .04)))
        pose = at(hit2 - reach * u)
        now = LocalPose(rig, pose)
        # The left palm stays on the moving haft; its station and roll on
        # the haft move from the contact grip to the guard grip.
        hand = now.world(WEAPON) @ blend(grip_contact, grip_guard, u)
        neck = now.world(rig.parents[left.names[0]])
        pose.update(steer_arm(contact_pose, guard_pose, left, hand, u, parent=neck))
        for n in left.fingers:
            if n in pose:
                pose[n] = blend(contact_pose.local(n), guard_pose.local(n), u)
        return pose

    from import_two_hand_viewmodel import turn_about
    from audit_staff_mesh_grip import matrix
    clavicles = [n for n in animated if n.endswith('Clavicle')]

    def aimed(pose, angles, weight):
        # Turn both clavicles by one rigid motion about the neck: the arms,
        # hands and spear move as one body and the shoulders stay seated.
        if not weight or not any(angles):
            return pose
        yaw, pitch, roll = (a * weight for a in angles)
        turn = turn_about((0., 0., 1.), yaw) @ turn_about((1., 0., 0.), pitch) @ turn_about((0., 1., 0.), roll)
        pose = dict(pose)
        now = LocalPose(rig, pose)
        for name in clavicles:
            parent = now.world(rig.parents[name])
            pivot = now.world(rig.parents[name])[:3, 3]
            motion = matrix(turn, pivot - turn @ pivot)
            pose[name] = inverse(parent) @ motion @ parent @ pose[name]
        return pose

    def drawn(e):
        # Weight of the stroke's own aim while drawing back.
        return 1. - ease(min(1., max(0., (e - .04) / (RETURN - .04))))

    def jab_for(angles):
        # Each stroke starts and ends in the guard (the drawn chamber), so a
        # series chains without the engine blending between far-apart poses:
        # settle, DM's drive and held extension, then draw back to guard.
        def jab(elapsed):
            if elapsed <= SETTLE:
                return guard_local
            if elapsed <= SETTLE + drive:
                return aimed(at(chamber + elapsed - SETTLE), angles, ease(min(1., (elapsed - SETTLE) / reach)))
            e = elapsed - SETTLE - drive
            return aimed(returning(e), angles, drawn(e))
        return jab

    cursor = max(float(t) for t, _ in text.keys) + 1.
    hit = SETTLE + hit2 - chamber
    length = SETTLE + drive + RETURN
    events = [(0., 'start'), (SETTLE - .04, 'plant'), (SETTLE, 'swing'),
              (hit, 'thrust hit'), (hit + .035, 'weight forward'), (hit + .07, 'settle end'), (length, 'stop')]
    for letter in 'abcd':
        for name in (('fasta', 'fastloopa') if letter == 'a' else ('fast' + letter,)):
            section(cursor, length, jab_for(AIM[name]), name, events)
            cursor += length + .2
        section(cursor, .05, lambda e: guard_local, 'fastend' + letter, [(0., 'start'), (.05, 'stop')])
        cursor += .25

    # Charged thrust: draw into the jab's chamber and hold it; release drives
    # it. The standing charge ("neutral") gets the same thrust: the engine
    # starts it on every click before it knows the click is a quick stroke,
    # and the staff's overhead start showed as a jump of the spear.
    chamber_local = at(chamber)
    holds = []
    total = drive + RETURN + .25
    for kind in ('thrust', 'neutral'):
        start, maximum, release = (marks[kind + ' ' + k] for k in ('start', 'max attack', 'min hit'))
        section(start, maximum - start,
                lambda e, s=maximum - start: {n: blend(guard_local[n], chamber_local[n], ease(e / s)) for n in animated},
                kind, [(0., 'start'), (marks[kind + ' min attack'] - start, 'min attack'), (maximum - start, 'max attack')])
        hold, hold_stop = marks[kind + ' hold start'], marks[kind + ' hold stop']
        holds.append((hold, hold_stop))
        section(hold, hold_stop - hold, lambda e: chamber_local, kind + ' hold',
                [(0., 'start'), (0., 'loop start'), (hold_stop - hold, 'loop stop'), (hold_stop - hold, 'stop')])
        jab = jab_for(AIM[kind])
        section(release, total, lambda e, jab=jab: jab(e + SETTLE),
                kind, [(0., 'min hit'), (0., 'plant'), (hit2 - chamber, 'hit'),
                       (hit2 - chamber + .035, 'weight forward'), (hit2 - chamber + .07, 'settle end'),
                       (drive + .1, 'chain')] +
                [(t, size + ' follow ' + edge) for t, edge in ((hit2 - chamber + .001, 'start'), (total, 'stop'))
                 for size in ('small', 'medium', 'large')])

    # Moving between the spear guard and a staff pose: DM regrips the arms
    # (upper arms differ by ~170 degrees of twist), so blending the bones
    # spins the hands and the spear in them. The spear moves on its own
    # world path instead; both palms follow it on their grips and the arms
    # reach them.
    grasps = {side: Grasp(contact_pose, side, False) for side in 'RL'}
    hand_bones = {side: grasps[side].names[-1] for side in 'RL'}
    arm_chain = {n for g in grasps.values() for n in g.names}

    from import_two_hand_viewmodel import quat_matrix
    from audit_staff_mesh_grip import matrix

    class Mix:
        """Blend two moving poses without flips. The rotation between a staff
        pose and the guard can be near 180 degrees, so the shortest way round
        changes side from one sample to the next. Each channel keeps the side
        it starts on for the whole transition."""
        def __init__(self, held=True):
            self.signs = {}
            self.held = held
            self.dip = np.array(DIP)

        def turn(self, key, a, b, w):
            qa, qb = matrix_quat(a[:3, :3]), matrix_quat(b[:3, :3])
            last = self.signs.get(key)
            if last is None:
                qb = qb if qa @ qb >= 0 else -qb
            else:
                qa = qa if qa @ last[0] >= 0 else -qa
                qb = qb if qb @ last[1] >= 0 else -qb
            self.signs[key] = (qa, qb)
            angle = np.arccos(np.clip(qa @ qb, -1., 1.))
            if angle < 1e-6:
                q = qa
            else:
                q = (np.sin((1 - w) * angle) * qa + np.sin(w * angle) * qb) / np.sin(angle)
            return matrix(quat_matrix(q[[1, 2, 3, 0]]), a[:3, 3] + w * (b[:3, 3] - a[:3, 3]))

        def __call__(self, a, b, w):
            w = min(1., max(0., w))
            out = {n: self.turn(n, a[n], b[n], w) for n in animated}
            if not self.held:
                return out
            A, B = LocalPose(rig, a), LocalPose(rig, b)
            spear = self.turn('spear', A.world(WEAPON), B.world(WEAPON), w)
            # The guard's hands sit beside the head; on the way to a staff
            # pose they pass under the eye instead of across it.
            spear[:3, 3] += self.dip * np.sin(np.pi * w)
            now = LocalPose(rig, out)
            for side, grasp in grasps.items():
                hand = hand_bones[side]
                grip = self.turn('grip ' + side, inverse(A.world(WEAPON)) @ A.world(hand),
                                 inverse(B.world(WEAPON)) @ B.world(hand), w)
                neck = now.world(rig.parents[grasp.names[0]])
                # The blended bones are both ends: steer_arm only reaches the palm.
                out.update(steer_arm(now, now, grasp, spear @ grip, 0., parent=neck))
                now = LocalPose(rig, out)
            out[WEAPON] = inverse(now.world(rig.parents[WEAPON])) @ spear
            return out

    # Side swing: from the guard straight into DM's wind-up, which keeps
    # running (slower, to the same max attack key); leave the recovery when
    # the point faces forward again and settle into the guard. The follow
    # stops move to the end of that settle.
    swing, wound = marks['slash start'], marks['slash max attack']
    joined = swing + SWING_JOIN
    section(swing, wound - swing,
            lambda e, mix=Mix(): mix(guard_local, at(joined + e / (wound - swing) * (wound - joined)), ease(min(1., e / JOIN))),
            'slash', [(0., 'start'), (marks['slash min attack'] - swing, 'min attack'),
                      (wound - swing, 'max attack')])
    leave = marks['slash hit2'] + SWING_LEAVE
    section(leave, LEAVE, lambda e, mix=Mix(): mix(at(leave + e), guard_local, ease(e / LEAVE)),
            'slash', [(LEAVE, size + ' follow stop') for size in ('small', 'medium', 'large')])

    # Draw and sheathe: DM's draw brings the haft over the shoulder into the
    # staff guard; the spear leaves that path for its own guard while it is
    # still coming down. Sheathing lets go of the spear at once (detach key),
    # so only the arms blend back into DM's sheathe.
    drawn_at, drawn_end = marks['equip attach'] - DRAW_EARLY, marks['equip stop']
    section(drawn_at, drawn_end - drawn_at,
            lambda e, mix=Mix(): mix(at(drawn_at + e), guard_local, ease(e / (drawn_end - drawn_at))), 'equip', [])
    sheathe, sheathe_to = marks['unequip start'], marks['unequip detach'] + .25
    section(sheathe, sheathe_to - sheathe,
            lambda e, mix=Mix(held=False): mix(guard_local, at(sheathe + e), ease(e / (sheathe_to - sheathe))),
            'unequip', [])

    # Sample and merge: replace keys inside every authored interval.
    samples = {}
    for begin, stop, fn in segments:
        for t in np.linspace(begin, stop, max(2, int(np.ceil((stop - begin) * RATE)) + 1)):
            samples[float(t)] = fn(float(t - begin))
    times = np.array(sorted(samples))
    for name in animated:
        controller = controllers[name]
        for index, field in enumerate((controller.data.rotations, controller.data.translations)):
            old = field.keys
            if old is None or not len(old):
                continue
            keep = np.ones(len(old), dtype=bool)
            for begin, stop, _ in segments:
                keep &= (old[:, 0] < begin - 1e-5) | (old[:, 0] > stop + 1e-5)
            values = [matrix_quat(samples[t][name][:3, :3]) if index == 0 else samples[t][name][:3, 3]
                      for t in times]
            merged = np.vstack((old[keep], np.column_stack((times, values)))).astype(np.float32)
            merged = merged[np.argsort(merged[:, 0], kind='stable')]
            if index == 0:
                for i in range(1, len(merged)):
                    if merged[i - 1, 1:] @ merged[i, 1:] < 0.:
                        merged[i, 1:] *= -1
            field.keys = merged
        controller.stop_time = max(controller.stop_time, cursor)

    replaced = ('fasta ', 'fastb ', 'fastc ', 'fastd ', 'fastend', 'fastloopa ', 'thrust ', 'neutral ')
    kept = []
    for t, label in text.keys:
        lines = [l for l in label.splitlines()
                 if not l.removeprefix('weapononehand: ').startswith(replaced)
                 and l.removeprefix('weapononehand: ') not in (
                     'slash start', 'slash min attack', 'slash max attack',
                     'slash small follow stop', 'slash medium follow stop', 'slash large follow stop')
                 and not (any(h <= t <= e for h, e in holds)
                          and l in ('weapononehand: loop start', 'weapononehand: loop stop'))]
        if lines:
            kept.append((float(t), '\r\n'.join(lines)))
    kept += labels + [(t, 'weapononehand: loop ' + edge) for h, e in holds for t, edge in ((h, 'start'), (e, 'stop'))]
    kept.append((0., 'weapononehand: brutal spear grip'))
    kept.sort(key=lambda p: p[0])
    text.keys = np.array(kept, dtype=text.keys.dtype)
    stream.save(target / source.name)
    # Strokes start where the previous one ended: keep only a short blend.
    import re
    rules = source.with_suffix('.yaml').read_text()
    rules = re.sub(r"(to: 'weapononehand:fast(?:end)?[abcd] start'\s+easing: linear\s+duration: )0\.2[0-9]*",
                   r"\g<1>0.045", rules)
    rules += "  - from: '*'\n    to: 'weapononehand:fastloopa start'\n    easing: linear\n    duration: 0.045\n"
    (target / 'weapononehand.yaml').write_text(rules)
    # Idle and gait: hold the spear guard in the arms and on the weapon; the
    # gait's own trunk, head and camera motion stay.
    arm_bones = [n for n in animated if n == ITEM or ((' L ' in n or ' R ' in n) and not n.endswith(('Thigh', 'Calf', 'Foot', 'Toe0')))]
    for path in sorted(folder.glob('*1h.kf')):
        if not path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            continue
        gait = load(path)
        gait_names = [e.string_data for e in gait.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
        for name, controller in zip(gait_names, gait.root.controllers):
            if name not in arm_bones or name not in guard_local:
                continue
            for index, field in enumerate((controller.data.rotations, controller.data.translations)):
                old = field.keys
                if old is None or not len(old):
                    continue
                value = matrix_quat(guard_local[name][:3, :3]) if index == 0 else guard_local[name][:3, 3]
                field.keys = np.column_stack((old[:, 0], np.tile(value, (len(old), 1)))).astype(np.float32)
        gait.save(target / path.name)
        if path.with_suffix('.yaml').exists():
            shutil.copyfile(path.with_suffix('.yaml'), target / path.with_suffix('.yaml').name)
    if body is not None:
        from import_spear_thrust import body_footwork
        body_footwork(body, target / source.name)
    manifest_path = arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())
    for path in sorted(target.iterdir()):
        manifest['outputs'][path.relative_to(arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['spear'] = {'base': 'brutal_fp_staff', 'jab': 'dm staff charged attack, second jab',
                         'chamber': JAB_CHAMBER, 'end': JAB_END, 'settle': SETTLE, 'return': RETURN, 'aim': AIM}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arms', type=Path, required=True)
    parser.add_argument('--body', type=Path)
    args = parser.parse_args()
    build(args.arms, args.body)
