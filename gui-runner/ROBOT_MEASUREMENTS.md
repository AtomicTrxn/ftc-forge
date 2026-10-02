# Robot measurements — calibration step 1

Open **Guided setup**, select your robot, and choose **Measure robot and verify motors…** from **Robot · Physics assumptions** or **Finish and use**. A team project is optional for entering measurements. Only sections you select are saved; Cancel preserves the current settings.

This step records manual measurements. Telemetry fitting and measured-versus-simulated response comparison are separate work; see [recording and fitting](CALIBRATION.md).

## 1. Measure operating weight

1. Assemble the robot as it will run: battery, electronics, wheels and mechanisms installed.
2. Remove game pieces. Weigh the whole robot on a scale.
3. Select **Use my measured operating weight**, choose **kg** or **lb**, and enter the scale reading.

The initial value comes from the prepared CAD/fallback masses or an existing total-weight override. Check it against the scale before confirming it as measured. The REV DUO starter base's CAD mass can omit operating components; do not treat that source value as a weighing of your complete robot.

Saving writes `runtime.total_mass_kg`. Runtime, scene preflight and native model preview/check use the same total mass, scaling prepared body masses and inertia tensors proportionally. This corrects total weight; it does not measure each component's mass, center of mass or inertia distribution.

## 2. Measure wheel dimensions

Select **Use my measured drive dimensions**, then choose **mm**, **cm** or **in**. Switching units converts the displayed values.

| Measurement | How to measure | Used for |
| --- | --- | --- |
| Wheel diameter | Across the outer tread through the axle center. Enter diameter, not radius. | Saved radius is half the entered diameter; drive speed calculations and differential tire support/traction use it. |
| Track width | Between left and right wheel centers. Use center-to-center spacing, not the outside width of the chassis or tires. | Differential turning and Mecanum turning calculations. |
| Wheelbase, Mecanum only | Between front and rear axle centers. | Mecanum turning calculations. |

For the standard six-wheel REV DUO differential robot, start with the center axle's left/right spacing. Skid-steering's effective turning track can differ from this geometric measurement and still needs motion recordings. Wheelbase is not an input to its current differential drive model.

The Mecanum guide applies one measured diameter to all four drive slots. If the diameters differ, retain the CAD-derived radii and inspect/edit the individual model geometry rather than asserting a common measurement. Without explicit measurements, the simulator retains cylinder radii and chassis-child wheel-center inference where available, then its established defaults.

Saving dimensions changes drive calculations. It **does not rescale CAD, rewrite collision cylinders, or move wheel joints**. Compare the new dimensions with the CAD/collision preview. Correct a mismatch in Size/orientation, Parts or Advanced settings before approving the model again. Differential dimensions remain in `runtime.drive`; explicit Mecanum overrides use:

```json
"drive_geometry": {
  "wheel_radius_m": 0.048,
  "track_width_m": 0.30,
  "wheelbase_m": 0.35
}
```

These example numbers are illustrative defaults. Enter your measurements. `drive_geometry` is for Mecanum; differential profiles use their existing `drive.wheel_radius_m` and `drive.track_width_m` instead.

## 3. Verify motor bindings

The guide lists each configured movable joint, its motor/servo name, signed ratio and whether that name appears in the selected project's hardware configuration. An unbound joint is listed explicitly. No project means the hardware check is deferred.

Passive mimic followers are labeled **Passive follower**, with their powered source joint. They do not need another motor binding. Other unbound joints may also be intentionally passive; confirm their role before assigning a motor.

1. Compare the listed name with the motor name in your project's Robot Configuration and identify the real shaft it powers.
2. Select **Save and open motor bindings**. The guide saves selected measurements and opens **Parts and movement**. It can open bindings while keeping all dimensions unchanged.
3. Choose the movable wheel/shaft part and use **Bind motor / servo…** to select an available project hardware name. Select a project first if mapping is deferred.
4. Check **Motor-to-joint ratio**. It is signed motor output-shaft turns per joint turn: a direct coupling is normally magnitude 1; additional sprockets/gearing between that output shaft and the joint change it. The motor preset already describes its internal gearbox. Do not count the internal gearbox again as a joint transmission.
5. Verify direction and the correct side. In **Physics assumptions → Runtime / calibrated settings**, differential drive has left/right motor names and physical shaft signs. These signs describe mounting orientation. TeamCode `Direction.REVERSE` is a separate setting. Use the native preview for geometry and a controlled low-power robot check for wiring/direction; matching names alone cannot establish either.

Default Mecanum uses four named slots: `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive`. Differential uses the names configured in `drive.left_motor`/`right_motor`; the REV preparation supplies `leftDrive`/`rightDrive`. Bind each driven joint to its corresponding motor, and inspect intake/other mechanism joints separately. Passive mimic followers can intentionally be unbound and coupled to a powered joint; confirm that arrangement in Advanced settings.

Selecting a hardware name through the binding dialog marks it **User supplied**. Its generated ratio 1 remains **Default — provisional** until you explicitly edit the value. Rebinding through that dialog creates a default ratio, so recheck any previously tuned gearing afterward. Tire mode currently requires wheel joints coupled 1:1 to a side motor's output shaft; arbitrary additional drivetrain gearing needs a supported preparation.

## Save, review and reuse

- Confirmed numeric measurements are recorded as **measured manually** in the model's provenance. Other CAD/default/user/calibrated values are preserved, including existing telemetry calibration.
- All selected numbers must be positive and finite. Invalid input keeps the form available for correction.
- Editing a saved reviewed model creates a separate draft. Its old revision remains available; the changed draft needs native checking and explicit review before use.
- Save a reviewed revision after checking assumptions and collisions. For a simulation, check the scene again and rerun project compatibility checks.
- Export the reviewed robot profile to carry the source/prepared assets, measured values, provenance and existing calibration together. Model reuse and migration retain these saved settings through the existing preparation workflow.

Friction, tire slip, rubber stiffness, retention and response remain provisional unless independently measured/calibrated. This guide does not certify agreement with the physical robot.
