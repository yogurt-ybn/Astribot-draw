# Astribot Light-painting Recommendation Catalog

Release: 2026-10-24-simulation-validated-v1. Language: English. Selectable patterns: 58.

## Recommendation instructions

Use this catalog to recommend existing light-painting designs from a user's requested subject, occasion, mood and style. Return exact pattern IDs. The application resolves IDs with pattern_lookup.json; do not invent file paths or IDs.

- Choose up to three relevant entries, placing the best subject match first. Explain the visual match briefly in the user's language.
- All entries are minimal, smooth line drawings, with one continuous path per arm. They are not filled illustrations, photorealistic images, text, calligraphy or detailed portraits.
- Mirrored pairs repeat one subject on both sides. Mixed pairs contain the two specific subjects listed. Preserve the complete composition; swapping arms, replacing half, scaling or rotating creates a new, untested job.
- A single-color light stick does not reproduce preview colors or multicolor rainbow bands. Color is a hardware choice, not a property guaranteed by these designs.
- Mark a suggestion as approximate if its subject or detail differs from the request. If no suitable option exists, return match=none and an empty list, or explicitly offer a labeled nearby theme. Do not describe an approximate silhouette as an exact match.
- Do not recommend 020-flame, 024-butterfly or any handwriting draft. They are excluded. 055-butterfly-blossom is a separately tested composition, containing one wing silhouette and one blossom; it is not a complete bilateral butterfly.
- Every listed entry completed official T1 simulation. This is not a hardware qualification or a promise that the same cached plan works from any starting pose. The execution application handles state and compatibility checks.
- Focus user-facing recommendations on the drawing and its suitability. Give execution details only when requested.

Suggested response structure:

```json
{"match":"exact","recommendations":[{"pattern_id":"049-moon-star-five","reason":"A crescent and star suit a quiet night-sky theme.","approximate":false}]}
```

## Composition and visibility

Left and right in descriptions mean the audience's view. Robot left appears on audience right. Each side draws simultaneously; a mixed pair is one complete two-arm pattern, not two independently swappable assets.

The central 50 cm band, canvas u=208..408, is an assumed visual occlusion by a person. It is not a no-draw zone or a validated person collision boundary. Roots or connecting arcs may enter it. Main motifs are placed at the sides. The lookup file records each arm's occluded-length fraction.

## Quick intent guide

- Love or affection: 001-heart, 051-heart-sparkle, 056-cat-heart.
- Birthday or party: 002-balloon, 003-bow, 004-crown, 060-music-sparkle.
- Night sky or magic: 005-star-five, 006-sparkle, 007-moon, 044-spiral, 049-moon-star-five.
- Spring or flowers: 014-tulip, 018-blossom, 019-daisy, 050-blossom-leaf, 055-butterfly-blossom.
- Autumn or foliage: 011-leaf, 012-ginkgo, 013-maple, 058-leaf-droplet.
- Cute animals: 027-fish, 030-cat, 031-rabbit, 032-bear, 056-cat-heart.
- Sea, travel or landscape: 026-gull, 037-sailboat, 053-fish-wave, 057-gull-mountain.
- Space or technology: 029-orbit, 035-rocket, 036-ufo, 045-bolt, 054-rocket-comet.
- Festivals or good wishes: 017-clover-four, 038-lantern, 039-bell, 047-shield, 059-lantern-blossom.
- Abstract flowing light: 028-halo, 033-open-curl, 034-double-wave, 042-wave, 043-curl, 044-spiral, 048-s-ribbon.

## Pattern index

Keywords are editorial matching cues. All entries share the line-art style above; simple/moderate describes visual complexity, not robot feasibility.

### Romance and Celebration (4)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 001-heart | Rounded Heart Pair | Two mirrored versions. A full heart outline with a soft notch and rounded tip. | love, romance, affection, Valentine, anniversary, cute | simple |
| 002-balloon | Balloon with Ribbon Pair | Two mirrored versions. An open balloon outline flowing into a short trailing ribbon. | birthday, party, celebration, playful, lighthearted | simple |
| 003-bow | Ribbon Bow Pair | Two mirrored versions. Two broad ribbon loops meet at a narrow waist. | gift, ribbon, romance, cute, celebration | simple |
| 004-crown | Rounded Crown Pair | Two mirrored versions. A crown silhouette with three broad, rounded peaks. | royal, princess, fairy tale, achievement, celebration | simple |

### Sky and Space (6)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 005-star-five | Five-point Star Pair | Two mirrored versions. A recognizable five-point star with softened corners. | star, night sky, wish, achievement, celebration | simple |
| 006-sparkle | Four-point Sparkle Pair | Two mirrored versions. A four-direction sparkle with long, smooth points. | sparkle, magic, shimmer, starlight, wish | simple |
| 007-moon | Crescent Moon Pair | Two mirrored versions. Inner and outer arcs form a rounded crescent outline. | moon, night, dream, quiet, gentle | simple |
| 008-comet | Comet Trail Pair | Two mirrored versions. One sweeping open arc suggests a comet tail; it has no separate star-shaped head. | comet, shooting star, trail, flowing, space | simple |
| 009-cloud | Soft Cloud Pair | Two mirrored versions. A cloud outline with three rounded upper lobes and a broad base. | cloud, sky, soft, cute, dreamy | simple |
| 010-rainbow | Rainbow Arch Pair | Two mirrored versions. One high, open arch suggests a rainbow; it is a single line rather than multiple colored bands. | rainbow, hope, sky, minimal, optimistic | simple |

### Nature and Plants (12)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 011-leaf | Sweeping Leaf Pair | Two mirrored versions. A tilted, full leaf outline without a separate central vein. | leaf, nature, plant, growth, organic, elegant | simple |
| 012-ginkgo | Ginkgo Leaf Pair | Two mirrored versions. A broad fan-shaped leaf joins a short, integrated stalk outline. | ginkgo, fan, autumn, botanical, elegant | simple |
| 013-maple | Maple Leaf Pair | Two mirrored versions. A rounded three-lobed maple silhouette joins a short stalk. | maple, autumn, leaf, nature, lively | simple |
| 014-tulip | Tulip Pair | Two mirrored versions. A three-tipped flower cup extends into an integrated stem outline. | tulip, flower, spring, romance, elegant | simple |
| 015-lotus | Closed Flower Bud Pair | Two mirrored versions. A smooth, unopened bud narrows at the top and widens at its base; it has no layered lotus petals. | bud, petal, flower, calm, botanical | simple |
| 016-clover-three | Three-leaf Clover Pair | Two mirrored versions. Three broad lobes form a continuous clover outline. | clover, luck, nature, cute, good wishes | simple |
| 017-clover-four | Four-leaf Clover Pair | Two mirrored versions. Four rounded lobes form a lucky clover outline. | four-leaf clover, luck, good wishes, rounded, nature | simple |
| 018-blossom | Five-petal Blossom Pair | Two mirrored versions. Five broad petals form one flower outline without a separate center. | blossom, cherry blossom, flower, spring, gentle | moderate |
| 019-daisy | Six-petal Daisy Pair | Two mirrored versions. Six shallow, rounded petals form a simple flower outline. | daisy, flower, spring, cheerful, playful | moderate |
| 021-droplet | Rounded Water Drop Pair | Two mirrored versions. A drop outline narrows above a wide, rounded lower body. | water, raindrop, fresh, clean, nature | simple |
| 022-mountain | Twin Mountain Peaks Pair | Two mirrored versions. Two unequal, rounded peaks rise above a shared base outline. | mountain, landscape, outdoors, adventure, scenery | simple |
| 023-mushroom | Mushroom Pair | Two mirrored versions. A broad mushroom cap and thick stalk form one outline. | mushroom, forest, fairy tale, cute, whimsical | simple |

### Animals and Wings (6)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 025-angel-wing | Spread Angel Wing Pair | Two mirrored versions. A broad upper sweep and softly scalloped lower edge form one spread wing. | angel, wing, freedom, protection, graceful | moderate |
| 026-gull | Gull in Flight Pair | Two mirrored versions. An open rising-and-falling line suggests a flying gull's wings. | gull, bird, flight, freedom, minimal, sea | simple |
| 027-fish | Rounded Fish Pair | Two mirrored versions. A fish body connects to a broad tail; eyes and separate fins are omitted. | fish, ocean, sea, aquatic, playful | simple |
| 030-cat | Cat Head Pair | Two mirrored versions. A rounded face and two broad pointed ears form a cat-head outline without facial details. | cat, kitten, pet, cute, playful | simple |
| 031-rabbit | Long-eared Rabbit Pair | Two mirrored versions. Two long, broad ears join a rounded rabbit-head outline without facial details. | rabbit, bunny, pet, cute, gentle | simple |
| 032-bear | Bear Head Pair | Two mirrored versions. Two rounded ears and a full face form a bear-head outline without facial details. | bear, teddy bear, cute, cozy, playful | simple |

### Abstract Light Trails (11)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 028-halo | Circular Halo Pair | Two mirrored versions. One complete circle forms a simple light halo. | circle, halo, ring, geometry, minimal, calm | simple |
| 029-orbit | Tilted Orbit Pair | Two mirrored versions. A slanted closed ellipse suggests an orbital path. | ellipse, orbit, space, geometry, minimal | simple |
| 033-open-curl | Open Curl Pair | Two mirrored versions. An open curling line makes less than one full turn. | curl, swirl, flow, abstract, light trail | simple |
| 034-double-wave | Double Undulation Pair | Two mirrored versions. An open wave makes one and a half broad oscillations. | wave, ripple, rhythm, lively, abstract | simple |
| 042-wave | Gentle Wave Pair | Two mirrored versions. One broad sinusoidal oscillation forms an open light trail. | wave, ocean, flow, calm, minimal | simple |
| 043-curl | Flowing Vertical Curl Pair | Two mirrored versions. An open line sways vertically with broad, flowing bends. | curl, ribbon, flow, elegant, abstract | simple |
| 044-spiral | Inward Spiral Pair | Two mirrored versions. An open spiral gradually winds inward while retaining a broad central radius. | spiral, galaxy, nebula, swirl, geometry | moderate |
| 045-bolt | Rounded Lightning Bolt Pair | Two mirrored versions. A zigzag lightning silhouette keeps its angular rhythm with rounded corners. | lightning, energy, power, speed, technology | simple |
| 046-diamond | Rounded Diamond Pair | Two mirrored versions. Four rounded corners form a simple diamond-shaped outline, without gemstone facets. | diamond, rhombus, geometry, minimal, elegant | simple |
| 047-shield | Guardian Shield Pair | Two mirrored versions. A broad upper edge tapers into a rounded lower shield tip. | shield, protection, guardian, strength, good wishes | simple |
| 048-s-ribbon | Sweeping S Curve Pair | Two mirrored versions. A broad, open S-shaped line suggests a flowing ribbon. | S curve, ribbon, flow, minimal, elegant | simple |

### Festive and Everyday Objects (7)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 035-rocket | Small Rocket Pair | Two mirrored versions. A pointed nose, body and side fins form a single rocket silhouette. | rocket, space, exploration, science, technology | simple |
| 036-ufo | Flying Saucer Pair | Two mirrored versions. A rounded dome and wide, flat body form a flying-saucer outline. | UFO, alien, space, science fiction, playful | simple |
| 037-sailboat | Sailboat Pair | Two mirrored versions. A tall sail and hull connect into one boat outline. | sailboat, boat, sailing, ocean, journey | simple |
| 038-lantern | Round Lantern Pair | Two mirrored versions. A lantern cap, round body and short, broad tassel form one silhouette. | lantern, festival, Chinese New Year, good wishes, traditional | simple |
| 039-bell | Festive Bell Pair | Two mirrored versions. A bell-shaped body connects to a short, broad clapper outline. | bell, Christmas, holiday, celebration, festive | simple |
| 040-music | Musical Note Pair | Two mirrored versions. A single musical-note silhouette joins its note head, stem and flag. | music, note, rhythm, dance, lively | simple |
| 041-cup | Warm Drink Cup Pair | Two mirrored versions. A cup body and handle form one outer silhouette; the handle has no separate inner hole. | cup, coffee, tea, cozy, everyday | simple |

### Paired Compositions (12)

| Pattern ID | Name | Visual description | Matching keywords | Complexity |
|---|---|---|---|---|
| 049-moon-star-five | Moon and Star | Audience left: five-point star. Audience right: crescent moon. Inner and outer arcs form a rounded crescent outline. A recognizable five-point star with softened corners. | moon, night, dream, quiet, gentle, star, night sky, wish, achievement, celebration | simple |
| 050-blossom-leaf | Blossom and Leaf | Audience left: sweeping leaf. Audience right: five-petal blossom. Five broad petals form one flower outline without a separate center. A tilted, full leaf outline without a separate central vein. | blossom, cherry blossom, flower, spring, gentle, leaf, nature, plant, growth, organic, elegant | moderate |
| 051-heart-sparkle | Heart and Sparkle | Audience left: four-point sparkle. Audience right: rounded heart. A full heart outline with a soft notch and rounded tip. A four-direction sparkle with long, smooth points. | love, romance, affection, Valentine, anniversary, cute, sparkle, magic, shimmer, starlight, wish | simple |
| 052-rainbow-cloud | Rainbow and Cloud | Audience left: soft cloud. Audience right: rainbow arch. One high, open arch suggests a rainbow; it is a single line rather than multiple colored bands. A cloud outline with three rounded upper lobes and a broad base. | rainbow, hope, sky, minimal, optimistic, cloud, soft, cute, dreamy | simple |
| 053-fish-wave | Fish and Wave | Audience left: gentle wave. Audience right: rounded fish. A fish body connects to a broad tail; eyes and separate fins are omitted. One broad sinusoidal oscillation forms an open light trail. | fish, ocean, sea, aquatic, playful, wave, flow, calm, minimal | simple |
| 054-rocket-comet | Rocket and Comet Trail | Audience left: comet trail. Audience right: small rocket. A pointed nose, body and side fins form a single rocket silhouette. One sweeping open arc suggests a comet tail; it has no separate star-shaped head. | rocket, space, exploration, science, technology, comet, shooting star, trail, flowing | simple |
| 055-butterfly-blossom | Butterfly Wing and Blossom | Audience left: five-petal blossom. Audience right: butterfly-wing silhouette. One connected upper-and-lower wing silhouette with a root near the center; it is one side of a butterfly, not a complete insect. Five broad petals form one flower outline without a separate center. | butterfly wing, spring, light, delicate, nature, blossom, cherry blossom, flower, gentle | moderate |
| 056-cat-heart | Cat and Heart | Audience left: rounded heart. Audience right: cat head. A rounded face and two broad pointed ears form a cat-head outline without facial details. A full heart outline with a soft notch and rounded tip. | cat, kitten, pet, cute, playful, love, romance, affection, Valentine, anniversary | simple |
| 057-gull-mountain | Gull and Mountains | Audience left: twin mountain peaks. Audience right: gull in flight. An open rising-and-falling line suggests a flying gull's wings. Two unequal, rounded peaks rise above a shared base outline. | gull, bird, flight, freedom, minimal, sea, mountain, landscape, outdoors, adventure, scenery | simple |
| 058-leaf-droplet | Leaf and Water Drop | Audience left: rounded water drop. Audience right: sweeping leaf. A tilted, full leaf outline without a separate central vein. A drop outline narrows above a wide, rounded lower body. | leaf, nature, plant, growth, organic, elegant, water, raindrop, fresh, clean | simple |
| 059-lantern-blossom | Lantern and Blossom | Audience left: five-petal blossom. Audience right: round lantern. A lantern cap, round body and short, broad tassel form one silhouette. Five broad petals form one flower outline without a separate center. | lantern, festival, Chinese New Year, good wishes, traditional, blossom, cherry blossom, flower, spring, gentle | moderate |
| 060-music-sparkle | Musical Note and Sparkle | Audience left: four-point sparkle. Audience right: musical note. A single musical-note silhouette joins its note head, stem and flag. A four-direction sparkle with long, smooth points. | music, note, rhythm, dance, lively, sparkle, magic, shimmer, starlight, wish | simple |

## Asset lookup and precomputed trajectories

pattern_lookup.json is the address registry. Its patterns object is keyed by full pattern ID; number_index maps the original three-digit numbers to IDs. Numbers retain gaps for excluded designs. Resolve assets paths relative to the directory containing that JSON file. No GitHub download URL is implied.

Each entry supplies source.svg, sdk-plan.npz and sdk-plan.validation.json. These are the exact SVG and optimized 125 Hz command trajectory that completed the original simulation. The small validation JSON is required by the existing SDK trajectory loader and must remain next to its matching NPZ. The lookup stores each file's SHA256.

The schedules use uniform arc-progress IK sampling and constrained arc-length feedrate timing. Configuration: joint ceiling 1 rad/s, tip cruise/ceiling 0.2 m/s, fixed simulation finger geometry with gripper feedback checks skipped, 20 ms command preview, and stop at drawing endpoint. The 2 mm drawing-accuracy reference is report-only.

The cached command trajectory includes the original approach plus drawing and is bound to its recorded measured start. The lookup stores required starting joints, auxiliary head/chassis positions and model revision. An arbitrary-start cache adapter is not included. For another start state, use the measured-start SVG planning pipeline. Loading an NPZ alone does not install the original SVG runtime guards.
