<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->
<!-- Source: JCIIOT 2026 case 1 SOP.docx; paragraphs=29; images=5; vlm=5 -->

# L1 Task - Blue Hollow Plastic Box Transport
Level: L1 (max 10 points)
Scene: factory_sorting_1_3fo3erfhisem

## Task
Transport a blue, hollow plastic box from Pick Station 2 to Place Station 3.

## Station Mapping
- Pick Station 2 = input_5
- Place Station 3 = output_4
- Target object: line_5_container_h01_near or line_5_container_h01_far

## Standard Workflow
1. Navigate to the pick station using `move`.
2. Pick the material using `pick_up` with both `target` and exact `object_name`.
3. Navigate to the place station using `move`.
4. Place the material using `place_down`.

## Visual / Layout Notes
The factory environment features multi-tiered assembly lines arranged in a linear depth configuration. Workstations consist of light-blue tabletops with white rectangular assembly fixtures. Obstacles may appear dynamically in the factory aisles; plan the shortest, safest path and adjust dynamically to avoid collisions.

## Safety and Scoring Notes
Collisions with obstacles or nearby equipment incur penalties and require immediate stoppage. A secure grasp is a prerequisite before moving. The material must remain stable during transport without violent shaking or dropping. Final placement must be precise, landing smoothly within the designated area without tipping over or exceeding boundaries.

## Planner Hints
- Use exact internal station names (`input_5`, `output_4`).
- Use exact object_name from the metadata/SOP (`line_5_container_h01_near` or `line_5_container_h01_far`).
- Do not add extra skills beyond the four-step workflow.
