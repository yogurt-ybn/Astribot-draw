# Third-party attribution

The simulation loads Astribot robot descriptions from https://github.com/Astribot-Dev/astribot_descriptions at pinned commit `8c651a1abbe2821824a692aa3a3aa4cbf88e4122`. `UPSTREAM_LICENSE` retains that repository's Apache License 2.0 text. Upstream mesh assets are downloaded separately and verified against Git LFS object hashes; they are not included in this source tree.

Collision geometry is derived at runtime from the upstream model's meshes, with explicit convex-hull contact pairs, documented exclusions, tools and runtime configuration changes. Reports include model geometry names and derived measurements. These changes do not imply endorsement by Astribot.

Python dependencies retain their respective licenses; installing this project does not relicense those packages. No blanket license has been assigned to newly authored project scripts or generated assets pending the owner's decision.
