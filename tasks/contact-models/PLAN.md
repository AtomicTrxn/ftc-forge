# Tire slip and flexible intake contact

## Objective
Replace the REV DUO speed-target drive and rigid/proximity intake with finite contact forces and flexible rubber-flap dynamics. Keep existing projects compatible through opt-in configuration.

## Implementation
1. Add longitudinal/lateral tire slip forces limited by a friction ellipse, with separate static/sliding friction, per-wheel omni/traction properties and ray-tested ground support. Feed equal reaction torque into each drive shaft so encoder motion and current reflect grip, slip, airborne wheels and obstacles.
2. Derive wheel rotational inertia from CAD, plus an explicit reflected motor inertia. Integrate motor damping and contact impulses implicitly at fixed physics ticks; report wheel surface speed, slip, contact force and supported-wheel count.
3. Model each double-ended intake flap as two segmented elastic beams anchored to the physical intake shaft. Spring/damped hinge constraints bend under collision load and transmit reactions to the shaft. Animate the CAD rubber mesh from the segment transforms. Allocate the flap mass once across its core and flexible segments.
4. Use physical game-piece contact for intake transport and ejection. Keep the game piece in Bullet; containment reporting must not teleport, freeze or remove it. Preserve the legacy proximity intake for projects that do not select flexible contact.
5. Add validated tire/flap config, update the preparation tool and prepare a separate local contact-model project/launcher. The export's roughly 6 milligram CAD flap masses are unsuitable for dynamic rubber; use an explicit, documented baseline mass override pending measurement.

## Validation
- Friction-limit and dissipative-slip unit tests.
- Native grip versus slippery/airborne/blocked-drive checks, encoder/current feedback and timestep stability.
- Native flap obstacle bending, recovery, shaft reaction, mass accounting and game-piece contact transport/ejection checks.
- Run the supplied CAD with real FTC TeamCode; inspect the deformed CAD rendering and telemetry.
- Full Java/Python regression suite plus Mecanum and legacy intake checks, then commit, push and merge a PR.

## Accuracy boundaries
Normal wheel load starts with a quasi-static distribution over supported wheels; dynamic tire deformation and full suspension are outside this iteration. Segmented beams approximate rubber bending, with finite thickness and configurable spring/damping parameters rather than material FEM. Friction, reflected motor inertia and rubber mass/stiffness remain explicit unmeasured baselines. Hardware weight/telemetry are still needed to establish accuracy.
