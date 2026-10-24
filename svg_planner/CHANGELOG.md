# Uniform version history

- v1: original finite torso families, preset torso initial pose; interrupted, not counted as pass/fail.
- v2: same canonical initial whole-body pose for every SVG, generic 18DOF RRT approach/return. All three ran: ginkgo actual simulation passed; koi rejected by velocity gate; wings rejected because first paired IK endpoints collide, then entry RRT fails. Results retained unchanged.
- v3: two global algorithm corrections before the all-three frozen batch: exact continuous cubic-spline derivative upper bound for timing (instead of 2001-point estimate); lazy deterministic paired-IK branch backtracking over unchanged seeds, with full-pair collision screening along the whole shared-progress path. Maximum169pairs. No SVG-specific branch, parameter, posture, seed, or geometry edit. First accepted pair proceeds to schedule; schedule/RRT failure currently advances torso family rather than returning to another pair.

Both v3 changes apply identically to all three inputs. The drawing torso families remain finite, so rejected cases are not proofs of physical impossibility. Actual simulation outcomes are separate from planning outcomes.
