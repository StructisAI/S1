# Verify on the arm before publishing / shipping

Items the plugin could not verify without hardware (from the plugin hand-off). Tick each
on a real S1A/S1L before the public release and before Batch 01 ships.

- [ ] **Joint signs** — every joint moves in the URDF's positive direction for a positive
      command; set `inverted_joints` for right and mirrored left arm if not.
- [ ] **Disabled motors answer MIT frames** — if not, the 0xCC refresh fallback engages;
      confirm positions are consistent between the two paths after calibration.
- [ ] **Zero pose** — the mechanical zero in the assembly instructions equals the URDF zero;
      calibration writes it as motor zero (0xFE, flash). Photograph the pose for the docs.
- [ ] **CAN adapters on macOS / Windows** (gs_usb / slcan) and real-bus timing at 30 Hz.
- [ ] **Gains / torque limits** are safe on the CNC-supported arm; XL330 model on the S1L
      is `xl330-m077` (or update the config).
- [ ] Record one 20-episode dataset, train ACT, replay — the README commands as written.
- [ ] Update `s1_description` inertials if measured values become available.
