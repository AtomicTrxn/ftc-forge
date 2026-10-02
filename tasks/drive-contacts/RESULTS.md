# Imported drivetrain contact results

## Delivered

Fresh general robot URDF/STL ZIP imports now select explicit native wheel support. Configured drive-wheel branches retain collision geometry in the welded chassis compound, with flattened V-HACD leaves mapped back to links and wheel joints. Mass/inertia remain counted once. Candidate generation preserves wheel/tread coverage even when the chassis already has a source proxy.

Native upward contacts on static supports provide a normal-load and material grip budget. Translation/yaw drive requests share a bounded budget. Aggregate translation has configurable rolling resistance. Unsupported wheels produce no drive impulse; native gravity, external pushes, scraping, normal collisions and obstacle friction remain active. Optional tires use native support/load and apply the same material-limited force to chassis and shaft reaction.

The guide offers **Wheel support model…**, provisional editable normal/gap/drag settings, wheel/chassis collision correction and native support diagnostics in movement review. Missing wheel colliders fail construction with the joint name. Existing saved profiles, identical-CAD reuse, migrated profiles and captured preparations retain their earlier contact mode until explicitly changed. Mode/tuning changes alter physical identity and require renewed review. Export/import/migration retain settings without editing old revisions or source CAD.

## Automated evidence

Commands:

```sh
./gradlew test :gui-runner:installDist
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew :gui-runner:verifyRobotPhysics --args='.local/robots/rev-duo-starter/retention-project' :gui-runner:verifyTorusRetention --args='.local/robots/rev-duo-starter/retention-project'
```

- Full Java suite: **135 cases** (GUI 103, physics 25, core 7), zero failures. GUI contains 12 native contact cases and existing drivetrain/mechanism, motion assessment, portable model, collision and preparation regressions.
- Preparation suite: **40 cases**, zero failures. Includes tuned contact bundle portability, changed-CAD preservation, immutable revisions, legacy preservation, fixed tread candidate generation and invalid support parameters.
- The unchanged supplied REV DUO retention preparation passes both native probes, including powered drive/intake and the retention probe's pickup/carry/stop/reverse-release and inactive/reverse/coasting/stalled/overload gates. This validates compatibility with its existing mode; it does not calibrate native contact mode on REV rubber.

| Native case | Evidence |
| --- | --- |
| Low-power drive at default friction | Approximately **0.209 m** forward, 2 supported wheels, no chassis support contact; chassis friction remains **0.6**. Removing the new mode reproduces essentially zero travel on the chassis proxy. |
| Drive capabilities | All 4 differential/tank and 6 Mecanum demo directions pass at default friction; powered mechanisms and targeted reviews still pass. |
| Airborne drive / external push | No invented planar/yaw traction. An external impulse continues carrying the airborne robot; gravity continues accelerating it downward. |
| Belly grounding | No supported wheels, scraping contacts reported, no clear travel; native chassis-floor friction remains active. |
| Obstacles | A solid wall blocks travel and retains native friction. Narrow obstacles that strike wheel geometry while missing the chassis retain wheel/obstacle friction; they are checked at first contact because separate posts permit steering around them. Wheel/floor tangential friction alone is transferred to the drive solver. |
| Material / timestep | Low floor grip bounds acceleration at 1/30, 1/120 and 1/480 s. Zero floor grip supplies no drive traction. Supported coasting loses more energy with a nonzero rolling resistance coefficient. |
| Optional tires | Finite powered travel with grip; essentially no travel and zero tire force with zero floor grip. Peak unloaded shaft speed exceeds the loaded result. |
| STL and transforms | Closed STL wheel shapes pass V-HACD, retain wheel identity after flattening and body rotation, support both wheels, move in the rotated forward direction and preserve total mass. |
| Persistence | An existing reviewed legacy model opts into a separate draft, preserves tuning when switched off/on, requires renewed review, and exports/imports the newly reviewed configuration. |

The drive fixture was corrected explicitly to use axle-aligned wheel cylinders and chassis clearance. Previous tests' zero-friction workaround was removed. A motion review invalidation test now changes friction to 0.8 rather than requiring a positive movement assessment after reducing it to nearly zero. Runtime still rejects “Correct” when native motion was not observed.

## Desktop evidence

Using the existing local Swing/OpenGL QA wrappers, Computer Use verified a fresh import, project motor choices, the wheel-support selector's instructions, editable runtime settings, guided collision correction and native movement review.

The original synthetic package has a 0.10 m chassis envelope and wheel collision axes inconsistent with the intended axle. It remains unsuccessful under native support: forward **−0.000055 m**, **0** supported wheels, **4** scraping contacts, and the guide displays the missing-support diagnosis.

Through the guide, both wheel proxies were explicitly set to cylinders with **π/2 roll**, and chassis proxy height to **0.06 m**. Contact gap was explicitly set to **0.001 m**. Friction stayed **0.6**; motor commands, bindings and generic fallback specifications stayed the same. The retest displayed movement observed, **0.208118 m** forward, **2** supported wheels and **0** scraping contacts. The guide showed a new observation requiring assessment, rather than approving the model automatically.

Private before/after reports are retained in `.local/desktop-qa/drive-contact-evidence/`. CAD, library revisions, project paths, wrappers and reports are excluded from the public commit. The synthetic package is engineering QA, not another team's measured robot.

## Remaining approximations

- Wheels remain welded collision proxies; this does not add detailed rotating wheels, Mecanum rollers, treads or suspension. The aggregate drive is still a speed-target approximation with a level chassis.
- Support/load is from the preceding native solve, giving a one-tick delay on new contact. The current model targets flat static support; inclined terrain and moving platforms require further work.
- Welded wheel/chassis surfaces share one body material. Independent rubber materials and measured traction curves remain future work. Aggregate translation rolling resistance is explicit and provisional; yaw rolling resistance is not independently modeled.
- Optional tire mode retains motor/shaft reaction; aggregate chassis-wall drive does not establish electrical stall current. The tire contact tolerance remains a legacy ray-support setting when native contact mode is disabled.
- CAD geometry, dimensions, wheel axes, mass/inertia and material values still need user review and measurements. This improvement establishes repeatable physical contact handling, not measured real-world accuracy.

## Implementation references

Contact callbacks and persistent-point methods were checked against the local Minie 9.0.3 API and [its ContactListener source](https://raw.githubusercontent.com/stephengold/Minie/9.0.3/MinieLibrary/src/main/java/com/jme3/bullet/collision/ContactListener.java). Restoration of native wheel/obstacle coefficients follows [Bullet's material combination implementation](https://raw.githubusercontent.com/bulletphysics/bullet3/master/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp).
