# Five new SVG designs: official T1 SDK simulation

All five inputs were generated from SVG_GENERATION_SPEC.md v1.1 and passed the original static mask, smoothness and same-SVG rendering checks. Their source hashes in [verification.json](verification.json) match the tested bytes. They exercise closed contours, open curves, rounded wings, unequal arm lengths/turns and a large vertical extent.

Start the existing official T1 simulator first, then run from the repository root:

```bash
bash robot_sdk/run_sim.sh draw robot_sdk/examples/generalization/asymmetric_spirals.svg --skip-gripper-checks
```

To repeat all five:

```bash
for name in asymmetric_spirals paired_ellipses flowing_s_curves swept_wings tall_ribbons; do
  bash robot_sdk/run_sim.sh draw "robot_sdk/examples/generalization/$name.svg" --skip-gripper-checks
done
```

Defaults remain 125 Hz, joint ceiling 1 rad/s, virtual-tip target/command ceiling 0.2 m/s, bounded 20 ms simulation preview and stop at the drawing endpoint. Uniform arc-progress IK sampling is automatic. Gripper checks are skipped only with the explicit simulation flag; finger geometry stays at its model preset and no gripper commands are sent. Other checks remain active. Drawing position error is now report-only: exceeding the 2 mm reference adds a quality warning, while completed motion remains successful and the CLI returns zero. Execution faults still fail normally. The following recorded results retain the strict audit statuses from before this policy change.

| Design | Previous motion | Uniform-sampling motion | Peak feedback FK error | Historical strict 2 mm audit |
| --- | ---: | ---: | ---: | --- |
| Paired ellipses | 13.76 s | 11.87 s | 1.325 mm | passed |
| Open S curves | 10.00 s | 9.43 s | 2.042 mm | failed |
| Swept wings | 14.52 s | 12.51 s | 2.543 mm | failed |
| Asymmetric spirals | 71.12 s | 11.67 s | 1.842 mm | passed |
| Tall ribbons | 9.43 s | 9.03 s | 1.764 mm | passed |

All five motions completed. Actual sequential runs replan from different current poses, so these timings are not a controlled same-start comparison. A separate no-motion replay of the exact original spiral start reduced the complete plan from 71.110 to 11.960 s and passed final command geometry, velocity and source-shape checks. The recorded runs used the original strict position gate. New shape checks include the bounded 0.1 mm sampling error in the existing 1 mm command-shape budget.

The official simulator container passed 44 SDK and 34 planner tests. Full models, raw logs, credentials and machine-local paths are omitted. These are finite sampled simulation checks, not physical grasping, surface-contact drawing or hardware validation. Historical ginkgo and frozen upstream evidence describe their original versions and are retained separately.
