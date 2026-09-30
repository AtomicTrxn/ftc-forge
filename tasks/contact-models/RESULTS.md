# Tire slip and flexible intake — validation results

## Implemented

- Opt-in differential tire contact: six ray-supported wheels, static/sliding friction ellipse, reduced omni lateral resistance, CAD wheel inertia, shared motor-shaft reactions, physical encoder/current feedback and wheel diagnostics.
- Six CAD rubber paddles deformed by 36 collidable elastic segments attached to the actual lower shaft. Finite contact stiffness/damping, mass transfer, shaft reaction and a validated mesh/shaft bending plane.
- Physical game pieces remain in Bullet. Containment is reported without proximity capture, teleportation, freezing or removal. The default contact-mode torus spawns flat above the floor.
- Existing tilted polycarbonate ramp gets a collision envelope from its native STL bounds. Thin boxes use a collision skin smaller than their half thickness.
- Contact-mode chain coupling constrains relative orientation, maintaining phase during paddle/game-piece loads. Attached paddle and follower inertia participate in implicit motor damping and numerical acceleration bounds.

## Evidence

All **58 Java tests** passed (5 SDK, 25 physics, 28 GUI/native integration). All **3 Python preparation tests** passed. Tests cover force limits and dissipation across signed slip directions, omni anisotropy, grounded/slippery/airborne/blocked shafts, timestep refinement, beam recovery, collision transport in both directions, shaft reaction, correct CAD mesh deformation plane, material validation and source preservation.

Representative native tire fixture, 2 seconds:

| Scenario | Travel | Shaft speed | Drive current | Peak slip | Supported tires |
| --- | --- | --- | --- | --- | --- |
| Grip, μ=0.9 | 1.803 m | 21.00 rad/s | ~0 A | 0.264 m/s | 6 |
| Slippery, μ=0.05 | 0.789 m | 20.43 rad/s | 0.151 A | 0.866 m/s | 6 |
| Airborne | 0 m | 21.00 rad/s | ~0 A | 0.945 m/s | 0 |
| Blocked, μ=3 test fixture | 0.270 m | 3.107 rad/s | 4.772 A | 1.049 m/s | 6 |

The blocked fixture deliberately uses high grip to isolate motor loading. Baseline friction remains unmeasured. Near-zero cruising current reflects the present lossless motor/gear/rolling approximation. Halving the timestep stays within the 8 cm travel tolerance.

The isolated beam loaded at its tip bent **0.827 rad** and recovered to approximately **0.000104 rad**. Native paddle collisions transported a free sphere approximately **+0.0219 m** and **−0.0508 m** in the two shaft directions. The shaft reacted to paddle loading. Mesh deformation preserves its rest coordinates and shaft width while bending in the radial/tangential plane.

Supplied 614-link / 750-visual CAD contact probe:

- Prepared mass **2.727383929318 kg**, including six explicit 5 g flap/core overrides; **0.027 kg** transferred into flexible bodies. Native assembled body mass matches, with the separate 100 g game piece accounted for.
- Three-second straight drive reached **x=1.1679 m**, lateral coordinate **0.00589 m**, yaw **−0.00592 rad**.
- Loaded intake shaft positions **13.48951 / 13.48952 rad**; phase difference approximately **0.000013 rad**.
- Six supported tire contacts; surface/hub slip approximately **0.0012–0.0029 m/s** at completion.
- Subsequent skid-steering turn reached **0.6741 rad**. Piece remains a physical body, finite and inside the field through contact and reversal.
- Illustrative torus moved from x≈0.42 to **1.4319 m** during approach, then **1.4717 m** after reversal. **Containment remained false**; this is contact/transport evidence, not validated capture.
- Heavy contact peaked at **1.603 rad**, above the configured 1.2 rad iterative stop. This numerical stop overshoot remains an explicit accuracy limitation under heavy loads.

Actual `RevDuoAuto` ran in the jME renderer with supplied CAD: drive encoders **1463 / 1463**, straight yaw **0.000187 rad**, final yaw **0.7445 rad**, final chassis **(0.7915, 0.0587, 0.0540) m**. The torus stayed on the floor at **(1.0460, 0.0320, −0.0755) m**, with containment false. Original rubber geometry visibly deforms; cores/shaft/wheels remain correctly placed. The inspected screenshot reports approximately **20 FPS**.

Legacy powered-CAD probe also passes: straight **x=1.2085 m**, yaw **−0.00178 rad**, proximity capture/release verified, differential turn **1.2224 rad**. Existing native mechanism regressions remain green. `BasicMecanumOpMode` also ran in the renderer: final chassis (0.8686, 0.1000, 0.7558) m confirms forward and lateral travel. `DashboardOpMode` completed normally with `Tuning.DRIVE_POWER=0.5`. `HungOpMode` was marked DEAD by the 500 ms watchdog and every motor power was zero afterward.

## Problems discovered and corrected

1. **Wrong paddle bending plane in the prototype.** The native mesh's width runs along X, not Y. The original assumption made segment bending perpendicular to the real shaft. Explicit mesh-to-beam coordinates now align width with the shaft, validated at import and by a mesh deformation regression.
2. **Light-segment constraint instability at 60 Hz.** Fast shafts and small segment inertias require finer integration. Contact mode now uses 480 Hz, 100 solver iterations and finite contact stiffness/damping. Heavy stop overshoot remains reported rather than described as exact rigidity.
3. **Artificial motor under-driving after splitting flap mass.** Actuator inertia originally included the remaining rigid core only. The bound/damping calculation now includes flexible mass inertia and its chain follower. Contact mode's numerical angular-acceleration cap is 1000 rad/s²; legacy behavior keeps 100 rad/s².
4. **Loaded chain phase drift.** Bullet's GearJoint constrains velocity, permitting accumulated position error under contact load. Contact mode instead locks relative shaft orientation while leaving translations free for the verified 1:1 parallel shafts.
5. **Missing intake ramp collisions.** The native polycarbonate plate existed only visually. Its source bounds and tilted link pose now determine a thin-box collision proxy.
6. **Initial torus/floor penetration.** The earlier upright torus at 5 cm height penetrated the floor and launched under free contact. Contact mode now places it flat at 3.4 cm before stepping physics.
7. **URDF containment bounds reversal.** URDF y maps to negative jME z, requiring reversed z bounds. Parsing now preserves the intended region and has a regression test.

## Remaining approximations

The modeling work is implemented, but hardware accuracy and successful intake retention are **not established**. Friction, motor inertia, flap mass/stiffness/damping/cross section and operating weight need measurement. Normal loads are equally distributed over flat static support; chassis pitch/roll and detailed omni rollers are absent. Rubber uses segmented bending/contact rather than material FEM. Heavy angular stops can overshoot. Gear losses, chain elasticity and real-season field/game-piece geometry remain outside this baseline. The illustrative torus is not captured by the current physical intake; no proximity behavior masks that result.

## Reproduction and local evidence

```sh
./gradlew test --offline --no-daemon
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew :gui-runner:verifyRobotPhysics --args='/absolute/contact-project'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/contact-project RevDuoAuto /absolute/contact.png'
```

The user's source export stays private and unchanged (SHA-256 `36b1106071e8d9e86b6866c9260fe65cc7e3923ea1a1e51376e1391f141d5195`). Native logs, test logs and the inspected screenshot are stored under local `.local/contact-models/`; private CAD and generated projects are excluded from Git. See [configuration and accuracy limits](../../gui-runner/CONTACT_MODELS.md).
