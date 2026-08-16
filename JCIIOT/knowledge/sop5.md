<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->
<!-- Source: JCIIOT 2026 case 9 SOP.docx; paragraphs=131; images=5; vlm=5 -->

# L5 Task - White-Rimmed Storage Bin Sorting
Level: L5 (max 30 points)
Scene: factory_sorting_9_3fo3ert2c5fp

## Task
Move the white-rimmed storage bins from Pick Station 6 to Place Station 1.

## Station Mapping
- Pick Station 6 = input_1
- Place Station 1 = aux_output_1
- Target object: white_tote_b01_left_center, white_tote_b01_left_front, white_tote_b01_left_back

## Standard Workflow
1. Navigate to the pick station using `move`.
2. Pick the material using `pick_up` with both `target` and exact `object_name`.
3. Navigate to the place station using `move`.
4. Place the material using `place_down`.

## Visual / Layout Notes
The environment features an elevated workstation or shelf with thick vertical support pillars underneath. These pillars act as physical obstacles that restrict lateral movement. A visual path indicator (green line) suggests a safe diagonal approach vector to the bins. Ensure the approach avoids the table legs and machine supports in the foreground.

## Safety and Scoring Notes
Collisions with obstacles or nearby equipment incur penalties. A stable, firm grasp is required before transit; if the grasp feels unstable, it is safer to reset than to risk dropping the item. Maintain a smooth, steady route during transport to avoid dropping the material. Final placement must be accurate, aligned, and within bounds—do not place crookedly or collide with other materials at the destination.

## Planner Hints
- Use exact internal station names (`input_1`, `aux_output_1`).
- Use exact `object_name` from the metadata/SOP.
- Do not add extra skills beyond the four-step workflow.
