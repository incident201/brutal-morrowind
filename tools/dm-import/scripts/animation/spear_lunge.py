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
    """Twist about the forearm, total bend, and the bend split into flexion
    (palm toward forearm) and sideways deviation, in degrees. The hand's +y is
    its palm normal: the calibrated shaft contact lies 3 units along it."""
    natural = fore[:3, :3] @ bind
    q = matrix_quat(natural.T @ hand)
    axis = natural.T @ fore[:3, 0]
    along = q[1:] @ axis
    twist = (np.degrees(2. * np.arctan2(along, q[0])) + 180.) % 360. - 180.
    swing_q = q[1:] - along * axis
    w = np.hypot(q[0], along)
    bend = np.degrees(2. * np.arctan2(np.linalg.norm(swing_q), w))
    palm = unit(np.array((0., 1., 0.)) - axis[1] * axis)
    lateral = np.cross(axis, palm)
    flex = np.degrees(2. * np.arctan2(swing_q @ lateral, w))
    deviation = np.degrees(2. * np.arctan2(swing_q @ palm, w))
    return twist, bend, flex, deviation


def strain(r, side):
    """Wrist and grip cost of a solved hand. The rig's bind pose is the
    anatomical neutral (palms facing in, thumbs up), so twist is pronation or
    supination with a human range of about 85 degrees each way. Sideways
    deviation is the narrowest range (about 20 radial, 30 ulnar). Walls make
    poses beyond those limits far costlier than any comfortable alternative.
    The front palm supports the haft from below: its palm faces up."""
    twist, flex, deviation = abs(r['twist']), abs(r['flex']), abs(r['deviation'])
    cost = (twist / 70.) ** 2 + (flex / 45.) ** 2 + (deviation / 18.) ** 2
    cost += (max(0., twist - 78.) / 6.) ** 2 + (max(0., flex - 55.) / 6.) ** 2 \
        + (max(0., deviation - 22.) / 4.) ** 2
    if side == 'L' and 'palm_up' in r:
        cost += (max(0., .4 - r['palm_up']) / .2) ** 2
    if 'reach' in r:
        # A locked straight arm reads as stiff; keep a little bend.
        cost += (max(0., r['reach'] - .93) / .02) ** 2
    return cost


def snap(x):
    """Accelerating drive: the point is fastest near contact."""
    x = float(np.clip(x, 0., 1.))
    return x * x * (2.6 - 1.6 * x)


def out(x):
    x = float(np.clip(x, 0., 1.))
    return 1. - (1. - x) ** 2.4


class LungeSpear(Spear):
    AIM = (5., 0., -115.)         # one line for every thrust, from the lower right
    REAR_STATION = -50.          # the rear fist sits at the butt cap (long thrust grip)
    SHAFT = (-53., 53.)
    # Visible body envelope in the guard camera frame. The body contract puts
    # the eye straight above the pelvis (37.5 units); the torso is an
    # elliptic cylinder under the eye, turning with the trunk.
    TORSO = dict(rx=11.5, rz=9., top=-12., bottom=-62.)
    POLES = {'R': (.35, -1., .45), 'L': (-.3, -1., .1)}   # elbows hang below and behind
    PHIS = np.radians(np.linspace(-50., 50., 21))
    TWIST_LIMIT, BEND_LIMIT = 115., 50.   # vanilla's own spear thrust twists the right wrist 92-108
    GUARD_REAR = (14., -34., -6.)
    FRONT_PALM = (8., -14.5, -23.)

    def __init__(self, rig, source, marks, native, fist, profile, single):
        self._ready = False
        self.native_rig = native.rig
        super().__init__(rig, source, marks, native, fist, profile, single)
        left = self.grasps[1]
        # Both hands use the skin-calibrated power grip of the staff import
        # (staff-native-grasp-profile.json): the palm contact point and the
        # shaft direction measured on the native hand mesh, with its closed
        # fingers; the right hand mirrors it. The contact point sits on the
        # shaft axis at station 0; grip() adds the station along the haft.
        self.base_grips, self.hand_fingers, self.anchors = {}, {}, {}
        for side, grasp in zip('RL', self.grasps):
            mirror = np.diag((1., 1., -1. if side == 'R' else 1.))
            axis = unit(mirror @ np.array(profile['shaft_axis_native_hand_reference']))
            anchor = mirror @ np.array(profile['shaft_anchor_native_hand'])
            rotation = swing(axis, np.array((0., 1., 0.)))
            self.base_grips[side] = matrix(rotation, -rotation @ anchor)
            self.anchors[side] = anchor
            fingers = dict(grasp.fingers)
            for name, value in profile['closed_finger_local_rotations'].items():
                name = name.replace(' L ', ' ' + side + ' ')
                fingers[name] = matrix(mirror @ np.array(value) @ mirror, fingers[name][:3, 3])
            self.hand_fingers[side] = fingers
        self.rear_options = {'profile': (self.base_grips['R'], self.hand_fingers['R'])}
        self.rear_fingers = self.hand_fingers['R']
        self.front_fingers = self.hand_fingers['L']
        left = self.grasps[1]
        self.open_fingers = {n: blend(self.front_fingers[n], self.open_hand.local(n), .6) for n in left.fingers}
        self.binds = {'R': self.bind_wrist, 'L': self.left_bind_wrist}
        self.idle_guard = LocalPose(self.rig, {n: self.idle.local(n) for n in self.names})
        self.GUARD = self.between(self.GUARD_REAR, self.FRONT_PALM, turn=-22.)
        self.hand_grips = {'R': self.base_grips['R'], 'L': self.base_grips['L']}
        self._ready = True
        self.choose_grips()
        self.GUARD = self.fit_rolls(self.GUARD, spin=150.)
        self.keypose.cache_clear()
        self.guard = self.solve(self.GUARD).transforms

    def grip_poses(self):
        return [self.GUARD, self.between((16., -36., -1.), (9., -19., -20.), turn=-24.),
                self.held((6., -16., -30.), self.AIM, 18., turn=-6., lean=11.)]

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
                cost = strain(r, side) + ((roll - around[i]) / spin) ** 2
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
        g[1, 3] += self.REAR_STATION + (0. if side == 'R' else (pose.front if pose.front is not None else 21.))
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
            twist, bend, flex, deviation = wrist_angles(bind, fore, hand[:3, :3])
            costs.append(strain({'twist': twist, 'flex': flex, 'deviation': deviation}, 'R')
                         + .25 * (phi / np.radians(50.)) ** 2)
        costs = np.array(costs)
        weights = np.exp(-6. * (costs - costs.min()))
        phi = weights @ self.PHIS / weights.sum()
        upper, fore, reached = hinge_arm(grasp, shoulder, wrist, phi, pole)
        hand = hand.copy()
        hand[:3, 3] = reached
        twist, bend, flex, deviation = wrist_angles(bind, fore, hand[:3, :3])
        report = {'reach': float(np.linalg.norm(wrist - shoulder) / (a + b)),
                  'gap': float(np.linalg.norm(wrist - reached)),
                  'elbow': float(np.degrees(np.arccos(np.clip(unit(fore[:3, 3] - upper[:3, 3])
                                                             @ unit(reached - fore[:3, 3]), -1., 1.)))),
                  'twist': float(twist), 'bend': float(bend), 'flex': float(flex),
                  'deviation': float(deviation), 'swivel': float(np.degrees(phi)),
                  'palm_up': float(hand[:3, 1] @ self.camera[:3, 1])}
        return (clavicle, upper, fore, hand), report

    # Oblique grips, measured offline against wrist limits over guard, chamber
    # and contact (lunge/t34, t35, t37): the haft crosses each palm at 45
    # degrees, the front palm cradles it from below.
    GRIPS = {'R': (0., -45.), 'L': (0., 45.)}     # (thumb flip, tilt in the palm plane)

    def oblique(self, side, flip, tilt):
        """The calibrated grip turned within the palm plane, about the palm
        normal through the contact point, as a real hand holds a shaft
        diagonally from the heel of the hand to the base of the index finger."""
        from import_two_hand_viewmodel import quat_matrix
        flipped = matrix(turn_about((1, 0, 0), flip), np.zeros(3)) @ self.base_grips[side]
        normal = flipped[:3, :3] @ np.array((0., 1., 0.))
        q = np.r_[normal * np.sin(np.radians(tilt) / 2.), np.cos(np.radians(tilt) / 2.)]
        return matrix(quat_matrix(q), np.zeros(3)) @ flipped

    def choose_grips(self, poses=None):
        """Set both oblique grips and wrap each hand's fingers around the haft."""
        straight = Pose(self.native_rig, {}, 0.)
        self.grip_choice, self.finger_clearance = {}, {}
        for side in 'RL':
            flip, tilt = self.GRIPS[side]
            grip = self.oblique(side, flip, tilt)
            inside = inverse(grip)
            fingers, clearance = fit_fingers({n: straight.local(n) for n in self.hand_fingers[side]},
                                             self.hand_fingers[side], side, (inside @ np.r_[0., 0., 0., 1.])[:3],
                                             inside[:3, :3] @ np.array((0., 1., 0.)))
            self.hand_grips[side] = grip
            self.grip_choice[side] = ('profile oblique', flip, tilt)
            self.finger_clearance[side] = clearance
            if side == 'R':
                self.rear_fingers = fingers
            else:
                self.front_fingers = fingers
        return self.hand_grips

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
            palm = inverse(item) @ chain[-1] @ np.r_[self.anchors[side], 1.]
            report[side]['palm'] = float(np.hypot(palm[0], palm[2]))
            report[side]['station'] = float(palm[1])
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
                cost = strain(r, side) + ((roll - pose.rolls[i]) / hold) ** 2
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
    # Every thrust runs on one line, from the lower right to the aim point.
    # The front palm stays at the bottom edge of the view throughout: in the
    # chamber only the rear fist draws back to the hip and the haft slides
    # through the cradling palm; on the drive the palm moves a little forward
    # with the haft while the rear fist and the trunk deliver it.
    def chamber(depth=1., turn=-26.):
        rear = lerp(np.array(S.GUARD_REAR), np.array((16., -37., 0.)), depth)
        front = lerp(np.array(S.FRONT_PALM), np.array((8., -15., -22.)), depth)
        return B(tuple(rear), tuple(front), turn=turn, lean=-3. * depth,
                 carrier=(0., -2.5 * depth, .6 * depth), nod=1.5 * depth)

    def extension(front, station, turn, lean, step, nod):
        return H(front, S.AIM, station, turn=turn, lean=lean, carrier=(0., step, -.25 * step), nod=nod)

    quick = {}
    for letter, (draw_t, hit_t, settle_t, recoil_t, stop_t, depth, front, station, turn, lean, step, nod) in {
            'a': (.13, .24, .31, .40, .56, .55, (9., -21., -32.), 19., -14., 10., 6., -2.),
            'b': (.12, .23, .30, .39, .54, .50, (8.5, -20.5, -31.5), 19., -16., 9., 5., -1.),
            'c': (.14, .25, .32, .41, .57, .60, (9.5, -21.5, -31.), 19.5, -12., 12., 6., -3.),
            'd': (.17, .29, .38, .48, .66, .85, (8., -20.5, -33.), 18., -10., 14., 11., -3.5),
    }.items():
        hit = extension(front, station, turn, lean, step, nod)
        over = S.along(hit, 1.2)
        settle = S.along(hit, -.6)
        back = S.mix(hit, guard, .55)
        path = [(0., guard, ease, 0.),
                (draw_t, chamber(depth, -22. - 6. * depth), out, .3),
                (hit_t - .02, over, snap, .35),
                (hit_t, hit, out, 0.),
                (settle_t, settle, ease, 0.),
                (recoil_t, back, ease, -.2),
                (stop_t, guard, ease, -.25)]
        quick[letter] = dict(path=path, draw=draw_t, hit=hit_t, settle=settle_t, stop=stop_t)

    deep = chamber(1., -28.)
    breath = B((16.3, -37.5, .6), (8.1, -15.3, -21.6), turn=-28.5, lean=-3.4,
               carrier=(0., -2.7, .65), nod=1.8)
    lunge = H((8., -20.5, -33.), S.AIM, 18., turn=-10., lean=15.,
              carrier=(0., 13., -3.4), nod=-4.)
    charged = dict(
        prepare=[(0., guard, ease, 0.), (.40, chamber(.75, -26.), out, .3), (.667, deep, ease, 0.)],
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



# --- finger fitting -----------------------------------------------------------

def finger_chains(fingers, side):
    chains = []
    for digit in range(5):
        names = [f'Bip01 {side} Finger{digit}{suffix}' for suffix in ('', '1', '2')]
        if all(n in fingers for n in names):
            chains.append(names)
    return chains


def fit_fingers(straight, closed, side, point, axis, radius=1.1, skin=.62, tip=.8):
    """Curl each finger along its own calibrated fist pattern until it wraps
    the shaft without entering it. Every joint of a finger moves the same
    fraction k of the way from the straight (bind) finger to the calibrated
    closed fist, so the curl stays natural and never bends backwards; k may
    exceed 1 slightly for a thinner part of the grip. `point` and `axis` give
    the shaft line in the hand bone's frame. Returns the fingers and the
    smallest bone-to-surface clearance."""
    fingers = {n: m.copy() for n, m in straight.items()}
    axis = unit(np.asarray(axis, float))
    point = np.asarray(point, float)
    target = radius + skin

    def distance(p):
        d = p - point
        return np.linalg.norm(d - axis * (d @ axis))

    def chain_points(names):
        world, points = np.eye(4), []
        for name in names:
            world = world @ fingers[name]
            points.append(world)
        end = points[-1] @ np.r_[fingers[names[-1]][:3, 3] * tip, 1.]
        return [m[:3, 3] for m in points] + [end[:3]]

    worst = np.inf
    for names in finger_chains(fingers, side):
        best = None
        steps = np.linspace(0., 1.35, 13)
        for spread in np.linspace(-15., 15., 7):
            for kb in steps:
                for km in steps:
                    for kt in steps:
                        base = _extrapolate(straight[names[0]], closed[names[0]], kb)
                        fingers[names[0]] = matrix(base[:3, :3] @ turn_about((0, 1, 0), spread), base[:3, 3])
                        fingers[names[1]] = _extrapolate(straight[names[1]], closed[names[1]], km)
                        fingers[names[2]] = _extrapolate(straight[names[2]], closed[names[2]], kt)
                        gaps = [distance(p) - target for p in chain_points(names)[1:]]
                        inside = sum(max(0., -g - .05) for g in gaps)
                        # Pads on the surface, nothing inside, a coherent curl.
                        cost = min(abs(g) for g in gaps[-2:]) ** 2 + 40. * inside ** 2 \
                            + .3 * ((kb - km) ** 2 + (km - kt) ** 2) + (spread / 40.) ** 2
                        if best is None or cost < best[0]:
                            best = (cost, spread, kb, km, kt)
        _, spread, kb, km, kt = best
        base = _extrapolate(straight[names[0]], closed[names[0]], kb)
        fingers[names[0]] = matrix(base[:3, :3] @ turn_about((0, 1, 0), spread), base[:3, 3])
        fingers[names[1]] = _extrapolate(straight[names[1]], closed[names[1]], km)
        fingers[names[2]] = _extrapolate(straight[names[2]], closed[names[2]], kt)
        worst = min(worst, min(distance(p) - target for p in chain_points(names)[1:]))
    return fingers, worst


def _extrapolate(a, b, k):
    """Rotation from a toward b by fraction k (k > 1 continues past b)."""
    q = matrix_quat(a[:3, :3].T @ b[:3, :3])
    angle = 2. * np.arctan2(np.linalg.norm(q[1:]), q[0])
    if angle < 1e-6:
        return a.copy()
    axis = q[1:] / np.linalg.norm(q[1:])
    return matrix(a[:3, :3] @ turn_about(axis, np.degrees(angle * k)), a[:3, 3])

if __name__ == '__main__':
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('arms', 'native-skeleton', 'native-animation'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--body', type=Path)
    args = parser.parse_args()
    build(args.arms, args.native_skeleton, args.native_animation, args.body)
