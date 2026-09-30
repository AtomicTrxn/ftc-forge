# REV DUO starter-base template

Use `tools/prepare_rev_duo.py` with the supported Onshape export to create a complete team project. This template deliberately has no `robot.urdf` or CAD assets until preparation. See [the guide](../REV_DUO.md).

Hardware names are a simulator template: `rightDrive` on motor port 0, `leftDrive` on port 1, `intake` on port 2, and `imu`. Replace them with the team's exported FTC XML, matching the presets, drive/intake config, transmissions and TeamCode names.

The two drive motors use HD Hex manufacturer's bare-motor constants transformed through the actual 4:1 × 5:1 UltraPlanetary cartridge reductions: `(76/21)*(68/13) = 18.9304:1`, not a literal 20:1. Torque uses a lossless gearbox baseline. The Core Hex intake uses 125 RPM, 3.2 Nm, 4.4 A and 288 output counts/revolution. Verify the physical cartridges and collect telemetry to calibrate losses and traction.

- `RevDuoAuto`: powers the intake, drives straight, then turns; exits automatically.
- `RevDuoTeleOp`: arrow keys drive/turn; Space runs intake; E reverses it. Drag to orbit and scroll to zoom. Close the renderer to stop.

The CAD mass is used until `total_mass_kg` is set to the measured operating weight. Collision proxies, aggregate planar traction and proximity capture remain approximations. The torus is an illustrative game piece.
