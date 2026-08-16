<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->
<!-- Source: JCIIOT 2026 case 5 SOP.docx; paragraphs=171; images=5; vlm=5 -->

# L3 Task - Blue Material Transfer Bin Handling
Level: L3 (max 20 points)
Scene: factory_sorting_5_3fo3ertpxeut

## Task
Transport the blue material transfer bin from Pick Station 1 to Place Station 2.

## Station Mapping
- Pick Station 1 = aux_input_1
- Place Station 2 = output_5
- Target object: blue_tote_b01_far_right, blue_tote_b01_near_right

## Standard Workflow
1. Navigate to the pick station using `move`.
2. Pick the material using `pick_up` with both `target` and exact `object_name`.
3. Navigate to the place station using `move`.
4. Place the material using `place_down`.

## Visual / Layout Notes
The workstation features a raised cyan-colored table surface supported by cylindrical legs. The blue bin rests centrally on this surface. The table legs act as floor-level obstacles; approach the bin from a clear angle to avoid base collisions. Ensure the gripper avoids the table's sharp edges and background structural frames during the pick and place motions.

## Safety and Scoring Notes
Safety takes precedence over efficiency. Avoid collisions with table legs, frames, or the bin itself, as collisions incur penalties. A successful, stable grasp is a prerequisite before moving. The bin must not shake violently or drop during transport. Final placement must be slow, stable, and accurately within the designated target zone without tilting or exceeding boundaries.

## Planner Hints
- Use exact internal station names: `aux_input_1` and `output_5`.
- Use exact object_name from the metadata/SOP: `blue_tote_b01_far_right` or `blue_tote_b01_near_right`.
- Do not add extra skills beyond the four-step workflow.
