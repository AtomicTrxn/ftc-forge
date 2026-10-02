# Imported drivetrain wheel support

Fresh robot URDF/STL ZIP imports enable **native wheel contacts**. The wheels provide actual collision support, and traction is limited by that support and the saved contact friction. A belly resting on the floor can still scrape; wheels in the air cannot drive the chassis.

## Guided setup

1. In **Parts and movement**, choose the drivetrain and motor bindings. The simulator identifies continuous joints driven by the configured drive motors; CAD shape alone does not establish a drivetrain.
2. In **Physics assumptions**, choose **Wheel support model…**. Existing reviewed models and captured projects keep their previous mode until you select native support. Switching saves a draft and requires collision review.
3. Select each wheel and the chassis in **Settings for**. Preserve source collisions or review generated proxies. Make the wheel collision axis match the axle, and check the wheel contact plane against the chassis underside. A source chassis collider does not automatically cover wheel support. Fixed tread children can supply a wheel's geometry. Missing wheel branch collisions fail native construction with the joint name.
4. Select **Runtime / calibrated settings** to tune the saved support parameters below. Select **Shared defaults** or the chassis body root to tune body material. **Wheel grip…** selects configured drive joints and supplies independent sliding friction for each wheel branch, including fixed tread children. Check **Inherit chassis friction** to remove an override. Restitution and native rolling/spinning coefficients remain properties of the welded body.
5. Run the motion demo. **View demo results** reports actual travel, supported wheels, chassis support contacts and simulated effective grip (wheel coefficient × contacted surface coefficient). With no wheel support, inspect geometry, axes, floor placement and belly clearance. **Guided correction… → Collision geometry** opens the wheel settings; retest after an explicit correction.
6. Inspect native collision outlines and save a reviewed revision. Export/import preserves the contact configuration. Changed CAD preserves tuning but requires renewed collision review; old movement assessments become historical when the physical digest changes.

Changing drive wheel radius affects speed conversion and tire calculations. It does **not** resize CAD or collision geometry. Raising the contact gap does not manufacture support for absent or incorrectly placed shapes.

## Saved parameters

```json
"drive_contacts": {
  "enabled": true,
  "min_support_normal_y": 0.7,
  "max_contact_gap_m": 0.003,
  "rolling_resistance_coefficient": 0.005,
  "wheel_friction": [
    {"joint": "left_wheel_joint", "friction": 0.9},
    {"joint": "right_wheel_joint", "friction": 0.9}
  ]
}
```

| Parameter | Units / range | Meaning |
| --- | --- | --- |
| `enabled` | boolean | Native wheel support when true; previous ballast/proxy behavior when false or absent. |
| `min_support_normal_y` | 0.5–1 | Minimum upward component of a static contact's normal. Walls and moving bodies supply no drive traction. |
| `max_contact_gap_m` | meters, 0–0.02 | Maximum native persistent-contact separation accepted as support. |
| `rolling_resistance_coefficient` | dimensionless, 0–1 | Aggregate drive translation drag as a fraction of wheel normal impulse, opposing motion without reversing it. Optional tire mode uses its own tire force model. |
| `wheel_friction` | optional list, coefficient 0–2 | One joint/friction entry per selected drive wheel. Unlisted wheels inherit the actual chassis friction, including a chassis material override. Duplicate or non-drive joint targets fail validation. Entries have effect only in native support mode. |

Defaults are provisional assumptions. Measure dimensions, mass, rolling resistance and friction on the actual robot/field before claiming physical accuracy. Saved wheel/floor friction is multiplied to bound tangential impulse; zero grip on either side supplies no drive force. An absent `wheel_friction` list preserves previous behavior.

## Measuring and retaining grip

Use the actual tread and field material. Measure normal load and horizontal force at sliding onset: effective grip is force / normal load (both in newtons). The saved wheel coefficient is effective grip / saved surface coefficient, which must be nonzero. For example, 9 N sliding force under 20 N normal load gives effective grip 0.45; with surface friction 0.6, enter wheel friction 0.75. This measurement identifies the pair, not two independent material properties. Keep the surface setting consistent when reusing this tuning.

In optional tire mode, the static/sliding tire curve is a second limit. Review both limits under **Runtime / calibrated settings**. Native normal load varies per wheel and timestep; a motion demo's grip value reports the simulated contact budget, not a physical measurement. Aggregate driving still shares grip across supported wheels and does not predict detailed asymmetric tire steering.

Saving a change creates/updates a draft and invalidates collision review and movement evidence. Export/import and identical CAD reuse preserve coefficients and provenance. Unique compatible CAD joint renames migrate automatically. A removed/ambiguous target offers current powered continuous joints or **Remove saved wheel grip override**; other wheel tuning and the native contact mode are retained. Review bindings and wheel geometry before approving changed CAD.

Legacy direct URDF projects can opt in with this configuration in `sim.config`. They must provide usable wheel collision shapes. Capturing a project retains its explicit configuration or its existing legacy mode; it does not silently alter a validated preparation.

## Physics scope

Drive-wheel convex shapes, including V-HACD mesh leaves, remain welded in the chassis compound. Native child indices retain the source link/joint identity and transformed principal-axis geometry. Assembly mass and inertia are counted once.

Bullet supplies normal support, collisions, gravity and external pushes. The drive solver replaces tangential response only for qualifying wheel/static-support contacts; chassis scraping, mechanism contacts and wheel/obstacle response remain native. Wheel/obstacle contacts use the selected wheel sliding coefficient, with the body's saved rolling/spinning coefficients; other chassis contacts use the chassis material. Persistent contacts that cease to qualify regain this material combination. The aggregate controller's translation and yaw requests share a finite material grip budget based on the previous native solve's normal impulse and contact lever arms. Contact onset therefore has a one-tick load delay.

With optional differential tires, native wheel contacts replace support raycasts and equal static wheel loads. Tire forces remain bounded by both the configured slip model and the native material grip. The same limited longitudinal force produces chassis impulse and opposing shaft torque.

The aggregate drive remains a speed-target approximation with a level chassis. This does not simulate detailed rolling wheel geometry, Mecanum rollers, treads, suspension, inclined-surface traction or moving platforms. Wheel friction and the aggregate drag coefficient are not inferred rubber measurements. Electrical stall loading remains represented by the optional tire/mechanism models, not by an aggregate chassis-wall speed target.

See [support implementation](../tasks/drive-contacts/PLAN.md), [support validation](../tasks/drive-contacts/RESULTS.md), [wheel grip plan](../tasks/wheel-grip/PLAN.md) and [wheel grip evidence](../tasks/wheel-grip/RESULTS.md).
