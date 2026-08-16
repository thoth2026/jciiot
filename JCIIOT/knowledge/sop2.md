<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->
<!-- Source: JCIIOT 2026 case 3 SOP.docx; paragraphs=179; images=5; vlm=5 -->

# L2 Task - Green Tote Transport
Level: L2 (max 15 points)
Scene: factory_sorting_3_3fo3errph7x9

## Task
Transport the green-rimmed storage bin from Pick Station 1 to Place Station 3.

## Station Mapping
- Pick Station 1 = input_6
- Place Station 3 = output_4
- Target object: green_tote_b01_upper, green_tote_b01_lower

## Standard Workflow
1. Navigate to the pick station using `move`.
2. Pick the material using `pick_up` with both `target` and exact `object_name`.
3. Navigate to the place station using `move`.
4. Place the material using `place_down`.

## Visual / Layout Notes
The environment is a cleanroom-style electronics factory floor with a light blue, unobstructed navigation path. The target material is a large green plastic tote/bin with reinforced corners. The primary obstacles include the material bin itself and the white metal support frames of transport carts or workstations. Maintain safe distances from background automated assembly lines and conveyor modules.

## Safety and Scoring Notes
Collision penalty is strictly enforced with a zero-tolerance policy for impacts. A successful grasp is a prerequisite for movement; ensure appropriate clamping force to avoid material deformation. Maintain material stability during transport with no significant shaking or dropping. Accurate final placement requires the tote to make full contact with the placement surface, remaining stable, un-tilted, and within boundaries.

## Planner Hints
- Use exact internal station names: `input_6` and `output_4`.
- Use exact object_name from the metadata/SOP: `green_tote_b01_upper` or `green_tote_b01_lower`.
- Do not add extra skills beyond the four-step workflow.
