<!-- AI-GENERATED FROM DOCX - DO NOT REPLACE WITH DEFAULT SOP -->

# Standard Operating Procedure (SOP)

Task ID: MT-MOBILE-001
Version: AI-DOCX-1.0

## Standard Transport Workflow

1. Navigate to Pick Station
2. Pick material with `pick_up` using both `target` and exact `object_name`
3. Navigate to Place Station with object held
4. Place material with `place_down` and confirm stable placement

## Task Coordinate Reference

| Level | Scene | Task | Pick Station | Object Name | Place Station |
| --- | --- | --- | --- | --- | --- |
| L1 | factory_sorting_1_3fo3erfhisem | Transport a blue, hollow plastic box from Pick Station 2 to Place Station 3. | Pick Station 2 = input_5 | line_5_container_h01_near or line_5_container_h01_far | Place Station 3 = output_4 |
| L2 | factory_sorting_3_3fo3errph7x9 | Transport the green-rimmed storage bin from Pick Station 1 to Place Station 3. | Pick Station 1 = input_6 | green_tote_b01_upper, green_tote_b01_lower | Place Station 3 = output_4 |
| L3 | factory_sorting_5_3fo3ertpxeut | Transport the blue material transfer bin from Pick Station 1 to Place Station 2. | Pick Station 1 = aux_input_1 | blue_tote_b01_far_right, blue_tote_b01_near_right | Place Station 2 = output_5 |
| L4 | factory_sorting_7_3fo3erfky9rn | Transfer the blue, hollow plastic box from Pick Station 5 to Place Station 2. | Pick Station 5 = input_2 | blue_container_h01_back_upper, blue_container_h01_back_lower | Place Station 2 = output_5 |
| L5 | factory_sorting_9_3fo3ert2c5fp | Move the white-rimmed storage bins from Pick Station 6 to Place Station 1. | Pick Station 6 = input_1 | white_tote_b01_left_center, white_tote_b01_left_front, white_tote_b01_left_back | Place Station 1 = aux_output_1 |

## CRITICAL pick_up Rules

- `pick_up` requires BOTH `target` and `object_name`.
- Use the exact object name from the current scene metadata or generated SOP.
- Execute exactly four steps: `move`, `pick_up`, `move`, `place_down`.
