# Astribot synchronous light painting

Fixed-base MuJoCo prototype coordinating a shared moving torso with two synchronized arms. Three agent-generated drawings are preserved as fixed-world targets. This repository includes code, agent instructions, drawing masks, input paths, compact measured results and collision-model audits.

[中文说明与复现步骤](README.zh-CN.md) · [Agent specification v2](agent-spec-v2/AGENT_DRAWING_SPEC.md) · [Reusable prompt](agent-spec-v2/AGENT_PROMPT.zh-CN.md)

![Three synchronized examples](synchronized-overview-zh.png)

## Scope and limitations

- Shared posture-guided torso motion plus compensating arm IK, not a general whole-body optimizer.
- Fixed chassis, synchronized approach/draw/return; human interaction excluded.
- Explicit collision checks use convex hulls and documented mechanical-adjacency exclusions. Sampled clearance is not a continuous-time proof or real-robot safety certification.
- Minimum sampled model clearances are only 2–4 mm. Do not deploy these trajectories directly on hardware.
- The current 570 × 471 canvas remains the earlier neutral-torso design subset, not the enlarged whole-body workspace.
- New drawings require seed IK and torso-posture selection; this is not a plug-and-play planner for arbitrary PNG/JSON inputs.

## Local checks (no model download needed)

Use Python 3.12 and install `requirements.txt` plus `agent-spec-v2/requirements.txt` in an isolated environment. Then:

```bash
python -m compileall -q .
python agent-spec-v2/test_validator.py
python agent-spec-v2/check_backend_inputs.py .
```

## Simulation

Follow the pinned model clone, LFS checksum download and run commands in [README.zh-CN.md](README.zh-CN.md). Set `ASTRIBOT_ASSETS` to an existing pinned descriptions checkout, or place it under `descriptions/`. Run one model at a time to limit memory use. `render_sync.py` needs regenerated full-rate `execution.npz` and expanded model XML; the committed 10 Hz records are summaries, not substitutes for the original 100 Hz checks.

## Results and provenance

`summary.json`, per-design `results.json`, collision pair/exclusion reports, injected-collision controls and the independent final audit describe the tested scope. Videos are accelerated replays of recorded actuator integration, not direct state-setting execution.

Upstream model: [Astribot-Dev/astribot_descriptions](https://github.com/Astribot-Dev/astribot_descriptions), commit `8c651a1abbe2821824a692aa3a3aa4cbf88e4122`. Model meshes are intentionally not vendored. The optional `asset_manifest.json` is also not committed: `download_assets.py` independently reads hashes from the pinned public Git LFS pointers, verifies the downloads, and regenerates this report locally. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and [UPSTREAM_LICENSE](UPSTREAM_LICENSE).

No new license for original project code has been selected here. The upstream license file applies to upstream material; it must not be interpreted as a blanket license for all original project content.
