# so101_actuation: the real SO-101 servos, for sims and for deployment

Part of [so101-bimanual-sim](https://github.com/CodCodingCode/so101-bimanual-sim), the bimanual SO-101 real-to-sim twin in MuJoCo. That repo has the full two-arm clamp runner (sim preview, joint feedback, safety guard) referenced below as `rig_tools/...`.

**The problem.** On our bimanual SO-101 rig, the kinematics and cameras were calibrated well. Even so, a grasp that worked perfectly in sim missed on the real arms by about 1 cm per side and lifted nothing. The cause was the motors, not the geometry:

- The Feetech STS3215 servos run LeRobot's soft position gain: **P = 16** (the factory value is 32), **I = 0**, deadband 1.
- Each joint therefore settles **1–3° short** of its target and stays there.
- At 25 cm of reach, that's about 1 cm at the fingertip.

The real joint ranges also differ from the sim's. The left wrist roll stops at 107°, so a sim plan asking for 163° got clipped silently. That run knocked a camera holder off.

Fixing the motor side made the same task work on the real arms on 2026-11-11. The fixes were:

- real joint ranges in the sim;
- stiffer servo gain during the run;
- a small, capped per-joint correction on the way in, frozen before contact.

## What's here

| File                              | What it is                                                                                                                                                                                                                                                                            |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `so101_profile.json`              | Per arm and joint: real range (read from the servos, offsets included), servo registers (P/D/I/deadband), torque limits, **measured shortfall** at three poses, the rig's joint offsets, ports and camera ids, and default tracking settings                                          |
| `motor_model/so101_dynamics.json` | MuJoCo servo model fitted to the real arms on 2026-11-10: kp, kv, force limit, friction, armature, time constant. Joint RMS dropped from 1.79° to 0.52° (0.54° on the held-out right arm). It underestimates the shortfall at full reach: about 1° in sim vs 2.6° real on elbow_flex. |
| `cameras/so101_camera_rig.json`   | **The calibrated camera rig in use on the robot** (identical hash to the laptop's copy): overhead and both wrist cameras, with intrinsics (fx/fy/cx/cy), OpenCV k1/k2 distortion and poses. Load with `pi_sim`'s `camera_rig=` or read it directly.                                   |
| `cameras/history/`                | Earlier camera fits, kept for reference. **Don't use them for new work.** The `sim_assets_*` copy has un-fitted wrist cameras.                                                                                                                                                        |
| `tracking.py`                     | Sim helpers (`apply_to_mujoco`, `apply_motor_model`), servo register access (`read_servo_profile`, `stiffen`, `restore`), and `TrackingCorrector`                                                                                                                                     |

### Measured shortfall

Degrees, commanded minus measured, once the target has been still. Positive means the joint stops below a target that's larger than where it started.

| Pose                      | Left arm, P16                      | Right arm, P16                      |
| ------------------------- | ---------------------------------- | ----------------------------------- |
| Hover (tips 11 cm up)     | 0.1, −0.1, **−2.6**, −0.9, −0.3    | 0.4, −0.6, **−2.6**, 0.2, −0.4      |
| Close (tips at the table) | **1.2**, −0.2, **−2.6**, −0.1, 0.3 | **−1.4**, −0.5, **−2.3**, 0.2, −0.3 |
| Hold (15 g cube, 4 cm up) | 0.9, **−2.1**, **−2.6**, 0.1, 0.3  | −1.1, **−3.2**, **−2.4**, 0.2, −0.2 |

Joint order: pan, lift, elbow, wrist flex, wrist roll. Elbow_flex is consistently about 2.5° short. Shoulder_lift sags under load.

## Use it in a sim or planner

```python
import sys; sys.path.insert(0, "/path/to/sim/so101_actuation")
import tracking as TR
prof = TR.load_profile()
TR.apply_to_mujoco(model, prof)     # joints + position actuators limited to the real ranges (3 deg inside)
TR.apply_motor_model(model, prof)   # fitted servo dynamics (if your model doesn't already have them)
```

Joint and actuator names are assumed to be `left_shoulder_pan` and so on. Pass `name=lambda side, joint: ...` if yours differ. For a single arm, use `sides=("left",)` and that arm's entry.

Why it matters: policies trained against stiff, unlimited joints learn to expect precision the real arm doesn't have. Train with the real ranges, and either the fitted motor model or the correction below at deployment.

## Use it at deployment

```python
access = {"left": TR.so101_real_access(left_arm), "right": TR.so101_real_access(right_arm)}
# LeRobot: TR.lerobot_access(bus)  (adapter UNTESTED here: run TR.read_servo_profile(access["left"]) first)
print(TR.read_servo_profile(access["left"]))      # what the servos really run with
TR.stiffen(access)                                 # P 16 -> 32, RAM only (EEPROM lock on): gone after power-off
corr = TR.TrackingCorrector(prof)
try:
    while running:
        target = policy_or_ik()                    # {side: joint degrees}
        cmd = corr.step(target, measured, active=not in_contact)
        if about_to_grasp: corr.freeze()           # keep the learned correction, stop integrating
        send(cmd)
finally:
    TR.restore(access)                             # P back to 16
```

- **When it learns:** it integrates only while the target has been still for a few ticks and `active` is true, capped at ±4° per joint.
- **Freeze before contact.** An integrator against an obstacle (cube, table, the other arm) just pushes harder. Our clamp freezes it when the squeeze starts.
- **If your targets come from a sim that has its own sag** (for example, replaying a sim trajectory), pass that sim's _measured_ joints as `reference=`. Then only the extra real-world shortfall is corrected. Correcting all the way to the ideal IK pose lifted our fingertips 7 mm above where the grasp was designed. The full working version, with sim preview, replay, joint feedback and safety guard, is `rig_tools/clamp_control.py` plus `rig_tools/real_clamp.py`.
- **Stiffer gain has a cost:** more force in a collision and a bit more jitter. Keep it per run, together with torque caps and a collision guard (`rig_tools/safety.py`).

## Limits

- It only corrects what the encoders can see. A wrong joint _offset_ reads as "on target" and stays wrong; fix that in calibration.
- The shortfall depends on pose and load (see the table). Treat the profile numbers as typical, not exact, and re-measure after changing gains, loads or calibration.
- The ranges and offsets are for this rig's two arms (COM5 left, COM6 right) with the current `rig.json`. If the offsets change, re-read the ranges with a dry check.
