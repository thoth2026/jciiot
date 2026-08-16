<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->
<!-- Source: JCIIOT 2026 case 7 SOP.docx; paragraphs=189; images=5; vlm=5 -->

# L4 Task - Blue Container Material Handling
Level: L4 (max 25 points)
Scene: factory_sorting_7_3fo3erfky9rn

## Task
Transfer the blue, hollow plastic box from Pick Station 5 to Place Station 2.

## Station Mapping
- Pick Station 5 = input_2
- Place Station 2 = output_5
- Target object: blue_container_h01_back_upper, blue_container_h01_back_lower

## Standard Workflow
1. Navigate to the pick station using `move`.
2. Pick the material using `pick_up` with both `target` and exact `object_name`.
3. Navigate to the place station using `move`.
4. Place the material using `place_down`.

## Visual / Layout Notes
The environment features elevated green platforms, conveyor systems, and industrial machines. Pick Station 5 is an upper staging area with blue lattice-style crates. Place Station 2 is a lower staging area or grey bin. Obstacles include bulky equipment housings, conveyor support legs, and drawer units. Ensure pathways are clear and navigate around support structures to safely approach the elevated pick area and lower place area.

## Safety and Scoring Notes
Safety takes priority over speed. Avoid collisions with equipment housings and conveyor legs, as collisions incur penalties. A successful and stable grasp is a prerequisite before transport; do not continue if gripping stability is uncertain. Carry the material smoothly without excessive shaking or dropping. Final placement must be slow, controlled, and accurately within the designated zone without tipping or overhang to secure maximum points.

## Planner Hints
- Use exact internal station names: `input_2` and `output_5`.
- Use exact object_name from the metadata/SOP: `blue_container_h01_back_upper` or `blue_container_h01_back_lower`.
- Do not add extra skills beyond the four-step workflow.
