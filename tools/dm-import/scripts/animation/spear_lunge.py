#!/usr/bin/env python3
"""Body-driven first-person spear thrusts in a two-handed chest guard.

Guard: the rear (right) fist rests in front of the lower chest, the front (left)
palm supports the haft just at the bottom edge of the view, and the point is
directed at the crosshair at fighting distance. A thrust is driven by the trunk:
the right shoulder turns through, the body lunges, and the rear fist pushes the
haft through the front palm, which stays where it is and lets the shaft slide.

Arms follow the vanilla Morrowind rule measured on every first-person clip: the
elbow is a pure hinge and all pronation lives in the wrist. Each hand keeps one
roll on the round haft for every pose; the elbow's swivel is chosen per frame
for the least wrist twist and bend. Bone lengths never change. `report` on each
solved pose records reach, elbow flexion, wrist twist and bend.
"""
from functools import lru_cache
from typing import NamedTuple

import numpy as np

from import_staff_native_grasp import swing
from import_two_hand_viewmodel import ITEM, Pose, basis, inverse, lerp, matrix, matrix_quat, turn_about, unit
from import_spear_thrust import LocalPose, Spear, blend, ease

CARRIER = 'Brutal Camera Carrier'
SPINE, HEAD = 'Bip01 Spine1', 'Bip01 Head'


class Lunge(NamedTuple):
    # Positions are in the guard camera frame (x right, y up, -z forward),
    # a body frame that does not follow the trunk's turn.
    rear: tuple            # rear fist grip centre on the shaft
    aim: tuple             # a point on the shaft's line ahead of the rear fist
    front: float | None    # station of the front palm ahead of the rear fist; None = released
    turn: float = 0.       # trunk yaw, degrees; positive brings the right shoulder forward
    lean: float = 0.       # trunk pitch forward, degrees
    carrier: tuple = (0., 0., 0.)   # whole-body offset (carrier local: x right, y forward, z up)
    look: float = .9       # fraction of trunk rotation the eye cancels
    nod: float = 0.        # extra eye pitch, degrees (negative looks down)
    rolls: tuple = (0., 0.)  # extra roll of the rear fist (with the whole spear) and the sliding front palm


def hinge_arm(grasp, shoulder, wrist, phi, pole):
    """Two-bone arm with a pure hinge elbow. phi swivels the elbow about the
    shoulder-wrist line, measured from `pole`. Returns upper, forearm, wrist point."""
    a, b = grasp.segments
    delta = wrist - shoulder
    length = np.linalg.norm(delta)
    axis = delta / length
    reach = min(length, a + b - 1e-3)
    x = (a * a - b * b + reach * reach) / (2. * reach)
    h = np.sqrt(max(0., a * a - x * x))
    p = unit(pole - axis * (axis @ pole))
    t = np.cross(axis, p)
    elbow = shoulder + axis * x + h * (np.cos(phi) * p + np.sin(phi) * t)
    reached = elbow + unit(wrist - elbow) * b
    normal = np.cross(elbow - shoulder, reached - elbow)
    if np.linalg.norm(normal) < 1e-6:
        normal = t
    upper = matrix(basis(elbow - shoulder, normal) @ grasp.upper_frame, shoulder)
    fore = matrix(basis(reached - elbow, normal) @ grasp.fore_frame, elbow)
    return upper, fore, reached


def wrist_angles(bind, fore, hand):
    """Twist about the forearm and remaining bend of the hand, in degrees."""
    natural = fore[:3, :3] @ bind
    q = matrix_quat(natural.T @ hand)
    axis = natural.T @ fore[:3, 0]
    along = q[1:] @ axis
    twist = (np.degrees(2. * np.arctan2(along, q[0])) + 180.) % 360. - 180.
    bend = np.degrees(2. * np.arctan2(np.linalg.norm(q[1:] - along * axis), np.hypot(q[0], along)))
    return twist, bend


def snap(x):
    """Accelerating drive: the point is fastest near contact."""
    x = float(np.clip(x, 0., 1.))
    return x * x * (2.6 - 1.6 * x)


def out(x):
    x = float(np.clip(x, 0., 1.))
    return 1. - (1. - x) ** 2.4


class LungeSpear(Spear):
    REAR_STATION = -46.          # the rear fist sits just above the butt cap
    SHAFT = (-53., 53.)
    # Visible body envelope in the guard camera frame. The body contract puts
    # the eye straight above the pelvis (37.5 units); the torso is an
    # elliptic cylinder under the eye, turning with the trunk.
    TORSO = dict(rx=11.5, rz=9., top=-12., bottom=-62.)
    POLES = {'R': (.35, -1., .45), 'L': (-.3, -1., .1)}   # elbows hang below and behind
    PHIS = np.radians(np.linspace(-50., 50., 21))
    TWIST_LIMIT, BEND_LIMIT = 115., 50.   # vanilla's own spear thrust twists the right wrist 92-108
    GUARD_REAR = (14., -30., -10.)
    FRONT_PALM = (8., -24.5, -23.)

    def __init__(self, rig, source, marks, native, fist, profile, single):
        self._ready = False
        self.native_rig = native.rig
        super().__init__(rig, source, marks, native, fist, profile, single)
        left = self.grasps[1]
        from import_two_hand_viewmodel import Grasp
        vanilla = Grasp(native, 'R', False)
        # Rear fist candidates: the fitted one-handed thrust fist and the
        # native two-handed wide-weapon grip, each with its own fingers.
        self.rear_options = {'power': (self.power_grip.copy(), dict(self.single_fingers)),
                             'wide': (vanilla.grip.copy(), dict(vanilla.fingers))}
        self.base_grips = {'R': self.power_grip.copy(), 'L': left.grip.copy()}
        self.rear_fingers = dict(self.single_fingers)
        self.front_fingers = dict(left.fingers)
        self.open_fingers = {n: blend(left.fingers[n], self.open_hand.local(n), .6) for n in left.fingers}
        self.binds = {'R': self.bind_wrist, 'L': self.left_bind_wrist}
        # Palm centre in each hand's frame: mean of the finger bases, pulled
        # slightly back toward the wrist. Its radial distance from the shaft in
        # the fitted grip is the reference for the palm-contact audit.
        self.palm_offset, self.palm_radius = {}, {}
        for side, grip, fingers in (('R', self.power_grip, self.single_fingers), ('L', left.grip, left.fingers)):
            bases = [fingers[f'Bip01 {side} Finger{d}'][:3, 3] for d in (1, 2, 3, 4)]
            self.palm_offset[side] = .8 * np.mean(bases, axis=0)
            centre = grip @ np.r_[self.palm_offset[side], 1.]
            self.palm_radius[side] = float(np.hypot(centre[0], centre[2]))
        self.idle_guard = LocalPose(self.rig, {n: self.idle.local(n) for n in self.names})
        self.GUARD = self.between(self.GUARD_REAR, self.FRONT_PALM, turn=-18.)
        self.hand_grips = {'R': self.base_grips['R'], 'L': self.base_grips['L']}
        self._ready = True
        self.hand_grips = self.choose_grips(self.grip_poses())
        self.GUARD = self.fit_rolls(self.GUARD, spin=150.)
        self.keypose.cache_clear()
        self.guard = self.solve(self.GUARD).transforms

    def grip_poses(self):
        return [self.GUARD, self.between((16., -34., -3.), (10., -26., -18.), turn=-24.),
                self.held((2., -16., -30.), (0., -1., -120.), 9., turn=-4., lean=9.),
                self.held((1., -14., -31.5), (0., 0., -120.), 8., turn=4., lean=13.)]

    def fit_rolls(self, pose, around=None, span=110., step=5., spin=55., sides='RL'):
        """Choose each hand's roll on the haft for this key pose (least wrist strain).
        `around` holds the guard's rolls: the spear spins little in the fist."""
        rolls = list(pose.rolls)
        around = (0., 0.) if around is None else around
        for i, side in enumerate('RL'):
            if side not in sides:
                continue
            best = None
            for roll in around[i] + np.arange(-span, span + 1e-6, step):
                trial = list(rolls)
                trial[i] = float(roll)
                r = self.solve_arms(pose._replace(rolls=tuple(trial)))[1][side]
                cost = (r['twist'] / 70.) ** 2 + (r['bend'] / 35.) ** 2 + ((roll - around[i]) / spin) ** 2
                if best is None or cost < best[0]:
                    best = (cost, roll)
            rolls[i] = best[1]
        return pose._replace(rolls=tuple(rolls))

    # --- pose helpers ------------------------------------------------------
    def axis(self, pose):
        return unit(np.array(pose.aim) - np.array(pose.rear))

    @staticmethod
    def held(front, aim, station, **changes):
        """Pose from the front palm's body position, the aim point and the hands' spacing."""
        front, aim = np.array(front, float), np.array(aim, float)
        axis = unit(aim - front)
        return Lunge(rear=tuple(front - axis * station), aim=tuple(aim), front=float(station), **changes)

    @staticmethod
    def between(rear, front, **changes):
        """Pose from both hands' body positions; the haft runs through them."""
        rear, front = np.array(rear, float), np.array(front, float)
        axis = unit(front - rear)
        return Lunge(rear=tuple(rear), aim=tuple(front + axis * 100.), front=float(np.linalg.norm(front - rear)), **changes)

    def along(self, pose, distance, **changes):
        """Push the rear fist along the shaft; the front palm stays put and the haft slides."""
        shift = self.axis(pose) * distance
        front = None if pose.front is None else pose.front - distance
        moved = pose._replace(rear=tuple(np.array(pose.rear) + shift),
                              aim=tuple(np.array(pose.aim) + shift), front=front)
        return moved._replace(**changes)

    def aimed(self, pose, aim):
        """Re-aim the haft about the front palm so that it keeps its contact."""
        front = np.array(pose.rear) + self.axis(pose) * (pose.front or 0.)
        axis = unit(np.array(aim) - front)
        rear = front - axis * (pose.front or 0.)
        return pose._replace(rear=tuple(rear), aim=tuple(aim))

    # --- trunk and camera --------------------------------------------------
    def trunk(self, pose):
        guard = self.idle_guard
        b = self.camera[:3, :3]
        turn = b @ turn_about((0, 1, 0), pose.turn) @ turn_about((1, 0, 0), -pose.lean) @ b.T
        spine = guard.world(SPINE)
        torso = matrix(turn, spine[:3, 3] - turn @ spine[:3, 3])
        result = {SPINE: inverse(guard.world(self.rig.parents[SPINE])) @ torso @ spine}
        head = torso @ guard.world(HEAD)
        keep = b @ turn_about((0, 1, 0), pose.turn * (1. - pose.look)) \
            @ turn_about((1, 0, 0), -pose.lean * (1. - pose.look) + pose.nod) @ b.T
        head[:3, :3] = keep @ guard.world(HEAD)[:3, :3]
        result[HEAD] = inverse(torso @ guard.world(self.rig.parents[HEAD])) @ head
        return result

    def item_frame(self, pose):
        rear = (self.camera @ np.r_[pose.rear, 1.])[:3]
        axis = self.camera[:3, :3] @ self.axis(pose)
        x = unit(np.cross(axis, self.camera[:3, 1]))
        return matrix(np.column_stack((x, axis, np.cross(x, axis))), rear - axis * self.REAR_STATION)

    # --- arms --------------------------------------------------------------
    def grip(self, side, pose):
        roll = pose.rolls[0 if side == 'R' else 1]
        g = matrix(turn_about((0, 1, 0), roll), np.zeros(3)) @ self.hand_grips[side]
        g[1, 3] = self.REAR_STATION + (0. if side == 'R' else (pose.front if pose.front is not None else 21.))
        return g

    def arm(self, side, body, hand, free=False):
        grasp = self.grasps[0 if side == 'R' else 1]
        clavicle = body.world(grasp.names[0])
        shoulder = (clavicle @ body.local(grasp.names[1]))[:3, 3]
        a, b = grasp.segments
        wrist = hand[:3, 3]
        # Protract the collarbone toward a hand the arm alone cannot reach.
        if np.linalg.norm(wrist - shoulder) > (a + b) * .97:
            socket = shoulder - clavicle[:3, 3]
            for k in np.linspace(0., 1., 31):
                arc = swing(socket, unit(lerp(unit(socket), unit(wrist - clavicle[:3, 3]), .5 * k)))
                if np.linalg.norm(wrist - clavicle[:3, 3] - arc @ socket) <= (a + b) * .97:
                    break
            clavicle = clavicle.copy()
            clavicle[:3, :3] = arc @ clavicle[:3, :3]
            shoulder = clavicle[:3, 3] + arc @ socket
        pole = self.camera[:3, :3] @ np.array(self.POLES[side])
        bind = self.binds[side]
        costs = []
        for phi in self.PHIS:
            upper, fore, _ = hinge_arm(grasp, shoulder, wrist, phi, pole)
            twist, bend = wrist_angles(bind, fore, hand[:3, :3])
            costs.append((twist / 70.) ** 2 + (bend / 35.) ** 2 + .25 * (phi / np.radians(50.)) ** 2)
        costs = np.array(costs)
        weights = np.exp(-6. * (costs - costs.min()))
        phi = weights @ self.PHIS / weights.sum()
        upper, fore, reached = hinge_arm(grasp, shoulder, wrist, phi, pole)
        hand = hand.copy()
        hand[:3, 3] = reached
        twist, bend = wrist_angles(bind, fore, hand[:3, :3])
        report = {'reach': float(np.linalg.norm(wrist - shoulder) / (a + b)),
                  'gap': float(np.linalg.norm(wrist - reached)),
                  'elbow': float(np.degrees(np.arccos(np.clip(unit(fore[:3, 3] - upper[:3, 3])
                                                             @ unit(reached - fore[:3, 3]), -1., 1.)))),
                  'twist': float(twist), 'bend': float(bend), 'swivel': float(np.degrees(phi))}
        return (clavicle, upper, fore, hand), report

    def choose_grips(self, poses, tilts=np.arange(-30., 31., 10.)):
        """One grip per hand for every pose: thumb direction and the shaft's
        diagonal across the palm (tilt). Each pose may roll the hands on the
        haft (fit_rolls); the grip itself never slides in the fist."""
        chosen = {}
        self.grip_choice = {}
        for side in 'RL':
            best = None
            options = self.rear_options if side == 'R' else {'support': (self.base_grips['L'], self.front_fingers)}
            for option, (source, fingers) in options.items():
                for flip in (0., 180.):
                    for tx in tilts:
                        for tz in tilts:
                            # Pivot about the contact point on the shaft axis so
                            # that the palm stays on the haft.
                            pivot = np.array((0., source[1, 3], 0.))
                            turn = turn_about((1, 0, 0), tx) @ turn_about((0, 0, 1), tz) @ turn_about((1, 0, 0), flip)
                            base = matrix(turn, pivot - turn @ pivot) @ source
                            self.hand_grips[side] = base
                            worst = 0.
                            for pose in poses:
                                fitted = self.fit_rolls(pose, sides=side, step=15., span=180., spin=1e9)
                                r = self.solve_arms(fitted)[1][side]
                                worst = max(worst, (r['twist'] / 70.) ** 2 + (r['bend'] / 35.) ** 2)
                                if best is not None and worst >= best[0]:
                                    break
                            if best is None or worst < best[0]:
                                best = (worst, base, fingers, (option, flip, float(tx), float(tz)))
            chosen[side] = best[1]
            self.hand_grips[side] = best[1]
            self.grip_choice[side] = best[3]
            if side == 'R':
                self.rear_fingers = best[2]
        return chosen

    def solve_arms(self, pose):
        result = self.trunk(pose)
        body = LocalPose(self.rig, {**self.guard_base(), **result})
        item = self.item_frame(pose)
        report = {}
        for side in 'RL':
            grasp = self.grasps[0 if side == 'R' else 1]
            free = side == 'L' and pose.front is None
            grip = self.grip(side, pose)
            hand = item @ grip
            if free:
                hand[:3, 3] = (self.camera @ np.r_[(-13., -31., -14.), 1.])[:3]
            chain, report[side] = self.arm(side, body, hand, free)
            # Radial distance of the palm centre from the shaft axis, versus the
            # fitted grip it came from (0 = palm still on the haft).
            palm = inverse(item) @ chain[-1] @ np.r_[self.palm_offset[side], 1.]
            report[side]['palm'] = float(abs(np.hypot(palm[0], palm[2]) - self.palm_radius[side]))
            parent = body.world(self.rig.parents[grasp.names[0]])
            for name, world in zip(grasp.names, chain):
                result[name] = inverse(parent) @ world
                parent = world
            if side == 'R':
                result[ITEM] = inverse(chain[-1]) @ item
                result.update(self.rear_fingers)
            else:
                result.update(self.open_fingers if free else self.front_fingers)
        return result, report

    def shaft_in_body(self, pose):
        """Deepest penetration (units) of the haft into the torso envelope; 0 = clear."""
        rear = np.array(pose.rear)
        axis = self.axis(pose)
        b = self.camera[:3, :3]
        turn = turn_about((0, 1, 0), pose.turn)
        T = self.TORSO
        worst = 0.
        for station in np.linspace(*self.SHAFT, 107):
            p = rear + axis * (station - self.REAR_STATION)
            q = turn.T @ p                      # into the turned trunk's frame
            if not T['bottom'] < q[1] < T['top']:
                continue
            r = np.hypot(q[0] / T['rx'], q[2] / T['rz'])
            if r < 1.:
                worst = max(worst, (1. - r) * min(T['rx'], T['rz']) + 1.1)
            elif (r - 1.) * min(T['rx'], T['rz']) < 1.1:      # shaft radius
                worst = max(worst, 1.1 - (r - 1.) * min(T['rx'], T['rz']))
        return float(worst)

    def guard_base(self):
        return {n: self.idle.local(n) for n in self.names}

    def solve(self, pose):
        result, report = self.solve_arms(pose)
        transforms = {**self.guard_base(), **result}
        carrier = transforms[CARRIER].copy()
        carrier[:3, 3] += np.asarray(pose.carrier)
        transforms[CARRIER] = carrier
        solved = LocalPose(self.rig, transforms)
        report['body'] = self.shaft_in_body(pose)
        solved.report = report
        return solved

    @lru_cache(maxsize=512)
    def keypose(self, pose):
        if not self._ready:
            return super().keypose(pose)
        return self.solve(pose)

    # Base-class hooks: the staff regrip targets self.guard, the gait overlay
    # calls strike() with small sway offsets around the guard.
    def strike(self, travel, push, across, height, release=0.):
        if not self._ready:
            return super().strike(travel, push, across, height, release)
        g = self.GUARD
        shift = np.array((across, height, -travel))
        return self.solve(g._replace(rear=tuple(np.array(g.rear) + shift),
                                     aim=tuple(np.array(g.aim) + shift))).transforms

    @staticmethod
    def mix(a, b, w, trunk_w=None):
        trunk_w = w if trunk_w is None else trunk_w
        def m(x, y, k):
            if x is None or y is None:
                return y if k >= .5 else x
            if isinstance(x, tuple):
                return tuple(lerp(np.array(x, float), np.array(y, float), k))
            return float(lerp(x, y, k))
        values = {}
        for name in Lunge._fields:
            k = trunk_w if name in ('turn', 'lean', 'carrier', 'nod') else w
            values[name] = m(getattr(a, name), getattr(b, name), k)
        return Lunge(**values)

    def pose_at(self, t, path):
        """path: [(time, Lunge, curve, trunk_lead)]; the curve shapes the segment ending at that key."""
        for (start, a, *_), (end, b, curve, lead) in zip(path, path[1:]):
            if t <= end + 1e-9:
                phase = float(np.clip((t - start) / (end - start), 0., 1.)) if end > start else 1.
                w = curve(phase)
                # The trunk leads (lead > 0) or trails the hands within the segment.
                tw = curve(float(np.clip(phase * (1. + lead), 0., 1.))) if lead >= 0 else \
                    curve(float(np.clip((phase + lead) / (1. + lead), 0., 1.)))
                return self.mix(a, b, w, tw)
        return path[-1][1]

    def refine(self, pose, span=30., step=5., hold=25.):
        """Per-frame relief of the wrists around the interpolated rolls."""
        rolls = list(pose.rolls)
        for i, side in enumerate('RL'):
            best = None
            for roll in pose.rolls[i] + np.arange(-span, span + 1e-6, step):
                trial = list(rolls)
                trial[i] = float(roll)
                r = self.solve_arms(pose._replace(rolls=tuple(trial)))[1][side]
                cost = (r['twist'] / 70.) ** 2 + (r['bend'] / 35.) ** 2 + ((roll - pose.rolls[i]) / hold) ** 2
                if best is None or cost < best[0]:
                    best = (cost, roll)
            # Parabolic refinement between grid points keeps successive frames smooth.
            rolls[i] = best[1]
        return pose._replace(rolls=tuple(rolls))

    def beat(self, t, path):
        return self.solve(self.refine(self.pose_at(t, path))).transforms


# --- clip authoring -----------------------------------------------------------

def author(spear):
    """Key poses and timing for every spear section. Times are seconds within a section."""
    S = spear
    guard = S.GUARD
    fit = lambda pose: S.fit_rolls(pose._replace(rolls=guard.rolls), around=guard.rolls)
    H = lambda *a, **k: fit(S.held(*a, **k))
    B = lambda *a, **k: fit(S.between(*a, **k))
    # The chamber cocks the point up and pulls the rear fist back to the hip;
    # the drive levels the haft into the line while both hands rise into view.
    lines = {
        'a': (0., -1., -120.),     # chest
        'b': (0., 11., -115.),     # face
        'c': (1., -15., -115.),    # belly
        'd': (0., -2., -125.),     # full lunge
    }
    def chamber(depth=1., turn=-20.):
        rear = lerp(np.array(S.GUARD_REAR), np.array((16., -34., -3.)), depth)
        front = lerp(np.array(S.FRONT_PALM), np.array((10., -26., -18.)), depth)
        return B(tuple(rear), tuple(front), turn=turn, lean=-3. * depth,
                 carrier=(0., -2.5 * depth, .6 * depth), nod=1.5 * depth)

    def extension(line, front, station, turn, lean, step, nod):
        return H(front, lines[line], station, turn=turn, lean=lean, carrier=(0., step, -.25 * step), nod=nod)

    quick = {}
    for letter, (draw_t, hit_t, settle_t, recoil_t, stop_t, depth, front, station, turn, lean, step, nod) in {
            'a': (.13, .24, .31, .40, .56, .55, (2., -16., -30.), 9., -4., 9., 6., -2.),
            'b': (.12, .23, .30, .39, .54, .50, (1.5, -13., -29.5), 9., -6., 7., 5., -1.),
            'c': (.14, .25, .32, .41, .57, .60, (3., -18.5, -29.5), 10., -4., 12., 6., -3.),
            'd': (.17, .29, .38, .48, .66, .85, (1., -14., -31.5), 8., 4., 13., 11., -3.5),
    }.items():
        hit = extension(letter, front, station, turn, lean, step, nod)
        over = S.along(hit, 1.2)
        settle = S.along(hit, -.6)
        back = S.mix(hit, guard, .55)
        path = [(0., guard, ease, 0.),
                (draw_t, chamber(depth, -14. - 8. * depth), out, .3),
                (hit_t - .02, over, snap, .35),
                (hit_t, hit, out, 0.),
                (settle_t, settle, ease, 0.),
                (recoil_t, back, ease, -.2),
                (stop_t, guard, ease, -.25)]
        quick[letter] = dict(path=path, draw=draw_t, hit=hit_t, settle=settle_t, stop=stop_t)

    deep = chamber(1., -26.)
    breath = B((16.3, -34.6, -2.4), (10.2, -26.4, -17.4), turn=-25.5, lean=-3.4,
               carrier=(0., -2.7, .65), nod=1.8)
    lunge = H((1., -14., -31.5), (0., -1., -125.), 8., turn=6., lean=14.,
              carrier=(0., 13., -3.4), nod=-4.)
    charged = dict(
        prepare=[(0., guard, ease, 0.), (.40, chamber(.75, -22.), out, .3), (.667, deep, ease, 0.)],
        release=[(0., deep, ease, 0.), (.185, S.along(lunge, 1.5), snap, .35), (.20, lunge, out, 0.),
                 (.30, S.along(lunge, -.8), ease, 0.), (.44, S.mix(lunge, guard, .5), ease, -.2),
                 (.80, guard, ease, -.25), (.95, guard, ease, 0.)],
        hit=.20, hold=[(0., deep, ease, 0.), (.67, breath, ease, 0.), (1.24, deep, ease, 0.)])
    return quick, charged


def build(arms, native_skeleton, native_animation, body=None):
    import hashlib
    import json
    import re
    from import_spear_thrust import (body_footwork, movement_overlay, staff_back_hand)
    from import_two_hand_viewmodel import Rig, keys, load, native_starts, tracks
    folder = arms / 'Animations/brutal_fp_staff'
    source = folder / 'weapononehand.kf'
    target = folder.with_name('brutal_fp_spear')
    target.mkdir(exist_ok=True)
    stream = load(source)
    text = keys(stream)
    marks = {line.removeprefix('weapononehand: '): float(t)
             for t, label in text.keys for line in label.splitlines()}
    native_rig, native_tracks = Rig(native_skeleton), tracks(native_animation)
    starts = native_starts(native_animation)
    vanilla = Pose(native_rig, native_tracks, starts['idle2w: start'])
    fist = Pose(native_rig, native_tracks, starts['idlehh: start'])
    channels = tracks(source)
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    profile = json.loads((arms / 'staff-native-grasp-profile.json').read_text())
    staff_back_hand(stream, channels, rig, vanilla, marks['thrust min hit'])
    stream.save(source)
    single = Pose(native_rig, native_tracks, starts['weapononehand: thrust hit'])
    spear = LungeSpear(rig, channels, marks, vanilla, fist, profile, single)
    quick, charged = author(spear)

    segments, labels = [], []

    def section(name, start, length, path, events):
        segments.append((start, start + length, lambda t, p=path: spear.beat(t, p)))
        labels.extend((start + t, 'weapononehand: ' + name + ' ' + event) for t, event in events)

    start, maximum, release = (marks['thrust ' + k] for k in ('start', 'max attack', 'min hit'))
    prep = [(t * (maximum - start) / .667, *rest) for t, *rest in charged['prepare']]
    section('thrust', start, maximum - start, prep,
            [(0., 'start'), (marks['thrust min attack'] - start, 'min attack'), (maximum - start, 'max attack')])
    end, hit = charged['release'][-1][0], charged['hit']
    section('thrust', release, end, charged['release'],
            [(0., 'min hit'), (0., 'plant'), (hit, 'hit'), (hit + .035, 'weight forward'),
             (.40, 'settle end'), (.62, 'chain')] +
            [(t, size + ' follow ' + edge) for t, edge in ((hit + .001, 'start'), (end, 'stop'))
             for size in ('small', 'medium', 'large')])
    hold = marks['thrust hold start']
    period = charged['hold'][-1][0]
    section('thrust hold', hold, period, charged['hold'],
            [(0., 'start'), (0., 'loop start'), (period * .54, 'breath'), (period, 'loop stop'), (period, 'stop')])
    labels.extend([(hold, 'weapononehand: loop start'), (hold + period, 'weapononehand: loop stop')])

    cursor = max(float(t) for t, _ in text.keys) + .2
    for letter in 'abcd':
        q = quick[letter]
        names = ('fasta', 'fastloopa') if letter == 'a' else ('fast' + letter,)
        for name in names:
            section(name, cursor, q['stop'], q['path'],
                    [(0., 'start'), (q['draw'] - .038, 'plant'), (q['draw'], 'swing'),
                     (q['hit'] - .005, 'thrust hit'), (q['hit'] + .035, 'weight forward'),
                     (q['settle'], 'settle end'), (q['stop'], 'stop')])
            cursor += q['stop'] + .06
        section('fastend' + letter, cursor, .2, [(0., spear.GUARD, ease, 0.), (.2, spear.GUARD, ease, 0.)],
                [(0., 'start'), (.2, 'stop')])
        cursor += .26

    # The staff swings and equip keep the base class's regrip into the new guard.
    def source_pose(time):
        pose = Pose(rig, channels, time)
        return {n: pose.local(n) for n in channels}

    def join(time, length, entering):
        def motion(t):
            progress = t / length if entering else 1. - t / length
            return spear.staff_edge(channels, time + t, progress)
        segments.append((time, time + length, motion))

    def staff_attack(name):
        s0, m0 = marks[name + ' start'], marks[name + ' max attack']
        change = .46
        def enter(t):
            if t < change:
                return spear.regrip(1. - ease((t - .085) / (change - .085)), spear.idle)
            u = (t - change) / (m0 - s0 - change)
            return source_pose(s0 + t - change * (1. - ease(u)))
        segments.append((s0, m0, enter))
        last_hit = marks.get(name + ' hit2', marks[name + ' hit'])
        stop = marks[name + ' large follow stop']
        tail = stop - last_hit
        def leave(t):
            if t < tail - change:
                return source_pose(last_hit + t + change * ease(t / (tail - change)))
            return spear.regrip(ease((t - tail + change) / change), spear.idle)
        segments.append((last_hit, stop, leave))
    for name in ('slash', 'chop', 'neutral'):
        staff_attack(name)
    join(marks['equip stop'] - .32, .32, False)
    join(marks['unequip start'], .24, True)

    replaced = ('thrust ', 'fasta ', 'fastb ', 'fastc ', 'fastd ', 'fastend')
    for t, label in text.keys:
        kept = [line for line in label.splitlines() if not line.removeprefix('weapononehand: ').startswith(replaced)
                and not (hold <= t <= marks['thrust hold stop'] and line in ('weapononehand: loop start', 'weapononehand: loop stop'))]
        if kept:
            labels.append((float(t), '\r\n'.join(kept)))
    labels.append((0., 'weapononehand: brutal spear grip'))
    labels.sort(key=lambda pair: pair[0])
    text.keys = np.array(labels, dtype=text.keys.dtype)

    samples = {}
    for begin, stop, fn in segments:
        for t in np.linspace(begin, stop, max(2, int(np.ceil((stop - begin) * 120)) + 1)):
            samples[float(t)] = fn(float(t - begin))
    times = np.array(sorted(samples))
    from audit_staff_mesh_grip import nif
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    for name, controller in zip(names, stream.root.controllers):
        for field, values in (
                (controller.data.rotations, [matrix_quat(samples[t][name][:3, :3]) for t in times]),
                (controller.data.translations, [samples[t][name][:3, 3] for t in times])):
            old = field.keys
            keep = np.ones(len(old), dtype=bool)
            for begin, stop, _ in segments:
                keep &= (old[:, 0] < begin - 1e-5) | (old[:, 0] > stop + 1e-5)
            merged = np.vstack((old[keep], np.column_stack((times, values)))).astype(np.float32)
            merged = merged[np.argsort(merged[:, 0])]
            if field is controller.data.rotations:
                for i in range(1, len(merged)):
                    if merged[i - 1, 1:] @ merged[i, 1:] < 0.:
                        merged[i, 1:] *= -1
            field.keys = merged
        controller.stop_time = max(controller.stop_time, times[-1])
    stream.save(target / source.name)
    if body is not None:
        body_footwork(body, target / source.name)
    rules = source.with_suffix('.yaml').read_text()
    rules = re.sub(r"(to: 'weapononehand:fast(?:end)?[abcd] start'\s+easing: linear\s+duration: )0\.2[0-9]*",
                   r"\g<1>0.045", rules)
    rules += "  - from: '*'\n    to: 'weapononehand:fastloopa start'\n    easing: linear\n    duration: 0.045\n"
    (target / 'weapononehand.yaml').write_text(rules)
    for path in sorted(folder.glob('*1h.kf')):
        if path.stem.startswith(('idle', 'walk', 'run', 'sneak', 'jump', 'brutal_sprint')):
            movement_overlay(spear, path, target)
    manifest_path = arms / 'weapon-families-import.json'
    manifest = json.loads(manifest_path.read_text())
    for path in (source, *sorted(target.iterdir())):
        manifest['outputs'][path.relative_to(arms).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest['spear'] = {'charged': 'thrust', 'quick': list('abcd'), 'base': 'brutal_fp_staff',
                         'guard': 'two-handed chest guard', 'grips': {k: list(map(str, v)) for k, v in spear.grip_choice.items()}}
    manifest_path.write_text(json.dumps(manifest, indent=1) + '\n')
    return spear


if __name__ == '__main__':
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('arms', 'native-skeleton', 'native-animation'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--body', type=Path)
    args = parser.parse_args()
    build(args.arms, args.native_skeleton, args.native_animation, args.body)
