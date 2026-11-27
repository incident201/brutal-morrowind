#!/usr/bin/env python3
"""Pose blending and two-bone arm steering shared by the polearm animation steps."""
import numpy as np

from import_staff_native_grasp import swing
from import_two_hand_viewmodel import Pose, inverse, lerp, matrix, matrix_quat, quat_matrix, slerp, unit


def ease(x):
    x = float(np.clip(x, 0., 1.))
    return x * x * (3. - 2. * x)


def blend(a, b, weight):
    q = slerp(matrix_quat(a[:3, :3]), matrix_quat(b[:3, :3]), weight)
    return matrix(quat_matrix(q[[1, 2, 3, 0]]), lerp(a[:3, 3], b[:3, 3], weight))


class LocalPose(Pose):
    def __init__(self, rig, transforms):
        super().__init__(rig, {}, 0.)
        self.transforms = transforms

    def local(self, name):
        return self.transforms[name] if name in self.transforms else super().local(name)


def relaxed_wrist(forearm, hand, grasp, amount):
    neutral = forearm[:3, :3] @ grasp.wrist_frame
    direction = hand[:3, 3] - forearm[:3, 3]
    natural = swing(neutral @ grasp.palm_along, direction) @ neutral
    return blend(hand, matrix(natural, hand[:3, 3]), amount)


def steer_arm(start, end, grasp, hand, w, relax=0., parent=None, fixed_socket=False, shoulder_phase=None):
    """Move a bent arm between two poses, meeting the hand target at both bone lengths."""
    clavicle, upper, fore, wrist = grasp.names
    C, U, F, H = [blend(start.world(n), end.world(n), w) for n in grasp.names]
    if fixed_socket:
        if shoulder_phase is not None:
            C = blend(start.world(clavicle), end.world(clavicle), shoulder_phase)
        if parent is not None:
            C[:3, 3] = (parent @ start.local(clavicle))[:3, 3]
        U[:3, 3] = (C @ start.local(upper))[:3, 3]
    a, b = grasp.segments
    delta = hand[:3, 3] - U[:3, 3]
    d = np.linalg.norm(delta)
    axis = unit(delta)
    shifted = axis * max(0., d - (a + b - .35))
    if fixed_socket and np.any(shifted):
        hand = hand.copy()
        hand[:3, 3] -= shifted
    else:
        C[:3, 3] += shifted
        U[:3, 3] += shifted
    d -= np.linalg.norm(shifted)
    x = (a*a - b*b + d*d) / (2*d)
    pole = F[:3, 3] - U[:3, 3]
    elbow = U[:3, 3] + axis*x + unit(pole - axis*(axis @ pole))*np.sqrt(max(0., a*a - x*x))
    upper_axis = unit(start.local(fore)[:3, 3])
    fore_axis = unit(start.local(wrist)[:3, 3])
    U[:3, :3] = swing(U[:3, :3] @ upper_axis, elbow - U[:3, 3]) @ U[:3, :3]
    F[:3, :3] = swing(F[:3, :3] @ fore_axis, hand[:3, 3] - elbow) @ F[:3, :3]
    F[:3, 3] = elbow
    if relax:
        hand = relaxed_wrist(F, hand, grasp, relax)
    if parent is None:
        parent = start.world(start.rig.parents[clavicle])
    return {clavicle: inverse(parent) @ C, upper: inverse(C) @ U,
            fore: inverse(U) @ F, wrist: inverse(F) @ hand}
