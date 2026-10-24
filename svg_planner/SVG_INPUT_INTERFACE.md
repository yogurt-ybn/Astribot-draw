# Astribot T1 SVG drawing contract v1

Generate vector centerlines, not a raster image. Fixed chassis, no humans. Two arms run simultaneously with a shared moving torso. This SVG is a geometric request, never a hardware-ready program.

## Exact accepted format

```xml
<svg xmlns="http://www.w3.org/2000/svg" width="616" height="560" viewBox="0 0 616 560">
  <path id="left" fill="none" stroke="blue" d="M 436 310 Q 442 314 436 318"/>
  <path id="right" fill="none" stroke="orange" d="M 180 310 Q 174 314 180 318"/>
</svg>
```

- Exactly two direct child paths, one `id="left"`, one `id="right"` (or explicit `data-arm`). No groups/editor layers in v1: export each layer as a direct path.
- Robot left arm occupies image right. Robot right arm occupies image left. Labels are anatomical, not screen positions.
- Supported commands: absolute/relative `M L C Q Z`; repeated parameter sets supported. One initial move per arm. No disjoint subpaths. `Z` explicitly closes; nothing closes automatically.
- No A/H/V/S/T commands, transforms, CSS/style, hidden elements, clipping, masks, text, embedded images, use, external links, scripts, XML declarations, DTDs or entities. Unsupported input is rejected, never silently ignored.
- Width616 height560, viewBox `0 0 616 560`, unitless or px only. No resizing. Stroke centerline is the requested trajectory; stroke width is decorative, not a filling instruction. `fill="none"` required if fill specified.
- Draw inside the respective `t1-canvas/mask-left.png` and `mask-right.png`. Pixels are continuous edge coordinates. Masks are finite sampled marginal supports, NOT feasibility proof for two simultaneous arms.
- Calibration: x=−.77+.0025u, y=−.42, z=1.89−.0025v metres. Tool light tip is local `[0,-.23,0]` m from link7. Position only; no constrained tip orientation or surface contact.
- Prefer smooth, sparse Bézier paths. No forks, sudden reversals, self crossings or sharp corners if avoidable. Geometry can pass static tests but fail continuous IK or clearance.

## Processing and preservation

Curves adaptively flattened with <=0.1 px (0.25 mm) curve-to-polyline bound; maximum polyline step1px. All flattened vertices contribute to the shared progress grid; numerically coincident progress knots within1e-12 are merged to stabilize interpolation, with path displacement bounded by length×1e-12 and recorded. Each arm is arc-length normalized independently, both share the same smooth time progress and start/end together. This is a NEW synchronization rule, not old exported JSON index timing.

No automatic scaling, moving, mirroring, stroke sorting, path simplification or closure. Zero-length segments may be normalized without geometric change and are counted. Bounds and masks are checked, including a conservative curve-error envelope. Joint-space interpolation is accepted only if every planned drawing timestep is within1mm of the flattened input; report records this additional distortion. This is not a claim of mathematically exact execution or continuous-time shape bounds.

## Output interpretation

`static passed`: syntax, geometry and sampled-mask envelope only.
`ik passed`: a continuous seeded arm IK branch under one shared prescribed torso candidate; not yet full-cycle clear.
`full_plan passed`: sampled collision/joint/velocity/geometry checks for approach, simultaneous drawing and return.
`simulation passed`: fresh actuator-driven MuJoCo execution finished without the checked failures. This does NOT authorize hardware execution.

Anything `not_run` remains unverified. Failures preserve SVG and give reasons. Candidate search exhaustion means this finite search failed, not a proof the design is physically impossible. Do not label statically accepted examples executable.

The v1 backend checks commanded joint velocity and position limits, but has no calibrated acceleration/jerk/torque-margin or singularity hard limits. These are explicitly unvalidated, rather than implied by a successful plan. Actual simulation additionally gates <=2mm maximum drawing tracking error and actual joint limits. Dynamic collision checks remain sampled and use the existing proxy exclusions.
Actual velocity is reported but not hard-gated. The .2rad/s schedule check is not an actual-speed certification.
