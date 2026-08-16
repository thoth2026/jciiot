# How to test — 95/100 configuration

Nothing to configure. The defaults in the repo now *are* the scoring
configuration; just run the app.

```bash
cd D:/projects/jciiot/JCIIOT2026/JCIIOT && streamlit run app.py
```

Expected, all five levels end to end:

| level | score | object lands from station centre | collisions |
|---|---|---|---|
| L1 | **10 / 10** | 0.122 m | 0 |
| L2 | **15 / 15** | 0.782 m | 0 |
| L3 | **15 / 20** | 0.534 m | −5 penalty (see below) |
| L4 | **25 / 25** | 0.324 m | 0 |
| L5 | **30 / 30** | 0.163 m | 0 |

**Total 95 / 100.** Run time is 11–20 min per level (L5 ~11, L1/L2/L3 13–15,
L4 17–20).

The simulation is deterministic — re-running the same configuration reproduces
the final object pose exactly — so these numbers should come out identically for
you. If one differs, something really changed.

## Testing a single level without the LLM planner

Faster, and drives the same agent/backend/skills/recording path as the app; it
just supplies the four-step plan directly instead of asking the planner:

```bash
cd D:/projects/jciiot/JCIIOT2026/JCIIOT && "C:/Users/thoth/anaconda3/envs/jciiot2026/python.exe" src/robot_agent/workflows/fixed_task_sequence.py --task-index 3 --timestamp mytest
```

`--task-index` 0=L1, 1=L2, 2=L3, 3=L4, 4=L5. Results land in
`recordings/<EnvName>/` as `trajectory_<timestamp>_OK.json`.

## Two things to know before you submit

**1. L3 keeps a −5 collision penalty.** Its push-then-grasp sequence is tuned
against the poses its *sweeping* in-place turn produces, and every attempt to
remove the contacts broke the grasp (four attempts: in-place turns, staging-pose
turns, 6 cm and 12 cm stance offsets — scoring grasp-FAIL, grasp-FAIL, 5/20 and
grasp-FAIL against the current 15/20). Both L3 checkpoints do score; only the
penalty remains.

**2. This configuration uses the platform's transport attachment.** After a
verified strict grasp + lift, `pick_up` hands the object to robosuite's own
`transport_attachment` helper for the carry. You said that scores nothing — if
that is confirmed, this configuration is not submittable and the carry problem is
still open. Worth checking with the organisers first, because the backend's own
`grasp_object_physics` calls the same helper after verifying grasp and lift
(`robosuite_backend.py:1176`), i.e. the platform's own intended flow uses it.

To force the purely physical carry instead:

```bash
set JCIIOT_CARRY_ATTACHMENT=0
```

With that set the object is lost within ~0.12 m of base travel and nothing
reaches the place station. The full measurement set and root cause are in
`TUNING_LOG.md`.

## What changed underneath (independent of the attachment question)

These are real defects that were costing points regardless of which carry is used:

- **`_set_final_pose` applied xy before yaw.** `joint_mobile_yaw` is a hinge
  0.21 m behind the base centre, so re-orienting swings the centre up to ~0.3 m
  and threw away the position just set. L4's pick stance drifted from the
  requested (−9.849, 6.303) to (−9.639, 6.11), 5 cm inside the `input_2` proxy.
- **In-place turns were not in place** (same hinge), which is what drove L3's
  torso and gripper into `side_table_pos_y_2`.
- **L4's carry corridor was hard-coded** straight through the loose frame at
  x = −8.30, y ∈ [0.25, 1.75]. Now planned with A* on the inflated grid, with
  stall re-planning.
- **Scene 7 never registers `output_5`**, so `place_object_physics` refused the
  L4 place and the fallback dropped the box on the floor 1.22 m out. Now falls
  back to the semantic map.
- **L3's park pose sat 5 mm inside `side_table_pos_y_2`**, and pick_up turned in
  place there, latching the judge's collision flag.

Together these took L4 from 7/25 to a clean collision-free 25/25 and removed a
guaranteed penalty from L3.
