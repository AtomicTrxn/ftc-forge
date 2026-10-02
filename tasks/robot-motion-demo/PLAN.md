# Guided robot motion demo

- [x] Add a robot Motion demo step before collision review, plus a replay entry from Finish for reused robots. Keep model-only preparation available and explain optional deferral when movement is not mapped yet.
- [x] Derive capabilities from effective saved drivetrain/joints/transmissions rather than guessing from meshes. Differential: forward/back and both turns. Configured Mecanum: those plus both strafes. Demonstrate powered continuous/hinge/slide/servo mechanisms one at a time; list passive, coupled and unbound joints with setup instructions.
- [x] Run low-power, short sequences through the existing motor, drivetrain, articulated-body, tire and flexible-intake physics on a neutral floor. Use selected project hardware/presets when supplied; otherwise disclose temporary generic motor assumptions. Do not compile/start TeamCode or write model review proofs.
- [x] Show the active movement, actual displacement/yaw/joint travel, stop/pause/replay controls, and a final observation summary. Flag no motion, wrong direction, setup failures and unstable physics. Report is a session artifact pinned to the exact model and hardware context; edits require replay.
- [x] Verify capability selection, binding/missing-setup behavior, joint bounds, actual native motion in both drivetrains and mechanisms, stopped commands, review independence and unchanged model/project files. Exercise the desktop demo with a synthetic robot and the real REV profile, and document results.

Implementation and verification complete. See [results](RESULTS.md). Shipment is recorded by the associated pull request.
