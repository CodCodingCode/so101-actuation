"""SO-101 actuation helpers: use the REAL servos' behaviour in sims, and correct it at deployment.

Why: on our rig the Feetech STS3215 servos run LeRobot's soft position gain (P 16, I 0). Each joint settles 1-3 deg
short of its target and stays there; at 25 cm reach that is ~1 cm at the fingertip. A policy or IK path that is exact
in sim then misses on the real arm (2026-11-11: a two-arm grab missed the cube by ~1 cm per side and lifted nothing).
Kinematics and cameras were fine; the gap was the motors. See README.md.

Sim side
    profile = load_profile()
    apply_to_mujoco(model, profile)            # real joint ranges (3 deg inside) on joints + position actuators
    apply_motor_model(model, profile)          # fitted kp / kv / force / friction / armature per joint

Deployment side (any policy that outputs joint targets, or IK of end-effector targets)
    access = {"left": so101_real_access(left_arm), "right": ...}    # or lerobot_access(bus) / your own read/write
    stiffen(access)                            # P 16 -> 32 for this run, RAM only (EEPROM locked)
    corr = TrackingCorrector(profile)
    every tick:
        cmd = corr.step(target_deg, measured_deg, active=not_in_contact)
        send(cmd)
    restore(access)                            # P back to 16 (also resets on power-off)

The corrector nudges each joint's command until the MEASURED joint reaches the target (capped, only while the target
is still and the arm is not in contact), then holds that correction. Freeze it before any contact (grasp, push,
press): an integrator against an obstacle just pushes harder.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
REG_P, REG_D, REG_I, REG_DEADBAND, REG_MIN_ANGLE, REG_TORQUE_LIMIT, REG_LOCK = 21, 22, 23, 26, 9, 48, 55


# ------------------------------------------------------------------ profile
def load_profile(path=None):
    return json.loads(Path(path or HERE / "so101_profile.json").read_text())


def apply_to_mujoco(model, profile, margin_deg=3.0, name=lambda side, joint: f"{side}_{joint}", actuators=True):
    """Limit the sim's arm joints (and their position actuators) to the real servo ranges, margin_deg inside.
    Planners / IK that respect model.jnt_range can then never ask for an angle a real joint cannot reach."""
    import mujoco
    changed = {}
    for side, arm in profile["arms"].items():
        for joint, (lo, hi) in zip(JOINTS[:5], arm["range_deg"]):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name(side, joint))
            if j < 0:
                continue
            cur = np.rad2deg(model.jnt_range[j])
            new = np.radians([max(cur[0], lo + margin_deg), min(cur[1], hi - margin_deg)])
            model.jnt_range[j] = new
            model.jnt_limited[j] = 1
            if actuators:
                a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name(side, joint))
                if a >= 0:
                    model.actuator_ctrlrange[a] = new
            changed[name(side, joint)] = np.round(np.rad2deg(new), 1).tolist()
    return changed


def apply_motor_model(model, profile=None, name=lambda side, joint: f"{side}_{joint}", sides=("left", "right")):
    """Fitted servo dynamics (motor_model/so101_dynamics.json) on position actuators named like the joints."""
    import mujoco
    dyn = json.loads((HERE / "motor_model" / "so101_dynamics.json").read_text())["joints"]
    for side in sides:
        for joint, p in dyn.items():
            a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name(side, joint))
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name(side, joint))
            if a < 0 or j < 0:
                continue
            model.actuator_gainprm[a, 0] = p["kp"]
            model.actuator_biasprm[a, 1] = -p["kp"]
            model.actuator_biasprm[a, 2] = -p["kv"]
            model.actuator_forcerange[a] = [-p["forcerange"], p["forcerange"]]
            model.actuator_forcelimited[a] = 1
            if "timeconst" in p and p["timeconst"] > 0:
                model.actuator_dyntype[a] = mujoco.mjtDyn.mjDYN_FILTEREXACT
                model.actuator_dynprm[a, 0] = p["timeconst"]
            d = model.jnt_dofadr[j]
            model.dof_frictionloss[d] = p["frictionloss"]
            model.dof_armature[d] = p["armature"]


# ------------------------------------------------------------------ servo register access
class ServoAccess:
    """Minimal register access for one arm (servo ids 1..6). Implement read / write for your bus, or use the
    adapters below."""

    def read(self, sid, addr, n):          # -> list of n bytes
        raise NotImplementedError

    def write(self, sid, addr, data):      # data: list of bytes
        raise NotImplementedError


def so101_real_access(arm):
    """Adapter for C:\\robot\\real\\so101_real.Arm (bus.read / bus.sync_write)."""
    class A(ServoAccess):
        def read(self, sid, addr, n):
            return list(arm.bus.read(sid, addr, n))

        def write(self, sid, addr, data):
            arm.bus.sync_write(addr, len(data), [(sid, list(data))])
    return A()


def lerobot_access(bus, motor_names=JOINTS):
    """Adapter for a LeRobot FeetechMotorsBus (recent API: bus.read(name, motor) / bus.write(name, motor, value)).
    UNTESTED on our rig: check with read_servo_profile() before relying on stiffen()."""
    names = {REG_P: "P_Coefficient", REG_D: "D_Coefficient", REG_I: "I_Coefficient", REG_LOCK: "Lock",
             REG_TORQUE_LIMIT: "Torque_Limit"}

    class A(ServoAccess):
        def read(self, sid, addr, n):
            v = int(bus.read(names[addr], motor_names[sid - 1], normalize=False))
            return [v & 0xFF, (v >> 8) & 0xFF][:n]

        def write(self, sid, addr, data):
            v = data[0] | (data[1] << 8 if len(data) > 1 else 0)
            bus.write(names[addr], motor_names[sid - 1], v, normalize=False)
    return A()


def read_servo_profile(access, ids=range(1, 7)):
    """What the servos are actually running with (read-only)."""
    out = {}
    for i, joint in zip(ids, JOINTS):
        out[joint] = {"P": access.read(i, REG_P, 1)[0], "D": access.read(i, REG_D, 1)[0], "I": access.read(i, REG_I, 1)[0],
                      "deadband": access.read(i, REG_DEADBAND, 2),
                      "torque_limit": (lambda b: b[0] | b[1] << 8)(access.read(i, REG_TORQUE_LIMIT, 2))}
    return out


def set_p_gain(access, value, ids=(1, 2, 3, 4, 5)):
    """Position gain, RAM only: the EEPROM lock is set first so the change is NOT saved (gone after power-off)."""
    for i in ids:
        access.write(i, REG_LOCK, [1])
        access.write(i, REG_P, [int(value)])
    return [access.read(i, REG_P, 1)[0] for i in ids]


def stiffen(accesses, profile=None, ids=(1, 2, 3, 4, 5)):
    p = (profile or load_profile())["tracking_defaults"]["P_run"]
    return {side: set_p_gain(a, p, ids) for side, a in accesses.items()}


def restore(accesses, profile=None, ids=(1, 2, 3, 4, 5)):
    p = (profile or load_profile())["tracking_defaults"]["P_rest"]
    return {side: set_p_gain(a, p, ids) for side, a in accesses.items()}


# ------------------------------------------------------------------ tracking correction
class TrackingCorrector:
    """Per-joint capped correction so the MEASURED joints reach their targets despite the servos' steady-state
    shortfall. Integrates only while active (no contact) and the target has been still for a few ticks; after
    freeze() it keeps the learned correction unchanged.

    reference: by default the corrector drives the measured joint to the TARGET itself (policies that expect the
    joint to go exactly where commanded). If your target was produced in a sim that has its own sag (e.g. a replayed
    sim trajectory), pass that sim's MEASURED joints as reference= so only the extra real-world shortfall is removed.
    """

    def __init__(self, profile=None, n_joints=5, sides=("left", "right")):
        t = (profile or load_profile())["tracking_defaults"]
        self.gain, self.cap = t["gain"], t["max_correction_deg"]
        self.still_ticks, self.still_eps = t["still_ticks"], t["still_eps_deg"]
        self.n, self.sides = n_joints, sides
        self.reset()

    def reset(self):
        self.corr = {s: np.zeros(self.n) for s in self.sides}
        self._prev, self._still, self.frozen = None, 0, False

    def freeze(self):
        self.frozen = True

    def step(self, target_deg, measured_deg, reference_deg=None, active=True):
        """target_deg / measured_deg / reference_deg: {side: >= n joint degrees}. Returns {side: command degrees}
        (extra entries such as the gripper pass through unchanged)."""
        tgt = {s: np.asarray(target_deg[s], float) for s in self.sides}
        ref = tgt if reference_deg is None else {s: np.asarray(reference_deg[s], float) for s in self.sides}
        if self._prev is not None:
            moving = max(np.abs(tgt[s][:self.n] - self._prev[s][:self.n]).max() for s in self.sides)
            self._still = self._still + 1 if moving < self.still_eps else 0
        self._prev = tgt
        out = {}
        for s in self.sides:
            if active and not self.frozen and self._still >= self.still_ticks:
                e = ref[s][:self.n] - np.asarray(measured_deg[s], float)[:self.n]
                self.corr[s] = np.clip(self.corr[s] + self.gain * e, -self.cap, self.cap)
            c = tgt[s].copy()
            c[:self.n] += self.corr[s]
            out[s] = c
        return out
