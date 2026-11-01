# Leaf planner → ICBM

Applies to NISSAN_LEAF and NISSAN_LEAF_IC with existing ICBM enabled and
stock longitudinal control. Other vehicles retain the existing target selection.
No new CAN sender, brake command, or changes to Nissan PPP interlocks.
The stopped-lead advisory is a separate branch and is not included here.

## Target selection

ICBM now reads longitudinalPlan.aTarget, the planner's final acceleration
selection (AI/e2e, lead MPC, or cruise). For aTarget <= -0.3 m/s²:

    planner_cap = max(OEM_minimum, vEgo + aTarget * 3 seconds)
    ICBM_target = min(longitudinalPlanSP.vTarget, planner_cap)

The horizon is a provisional conversion to a set-speed cap, not a prediction
that the OEM ACC will achieve the requested acceleration. Targets use existing
metric/imperial rounding and minima (30 km/h or 20 mph).

Existing vision/map curve and speed-limit reductions remain upper bounds.
AI deceleration on a curve can further reduce the target, but does not replace
curvature-based speed selection. On request release the existing ICBM state
machine can increase toward the user's existing cruise/curve/limit target.

Both plan messages must pass message checks and have timestamps no older than
0.25 seconds; the source model timestamp in longitudinalPlan must also be fresh.
Invalid/nonfinite input inhibits all ICBM button requests, including increases,
instead of falling back to the higher cruise target. Recovery uses the existing
0.4-second activation delay. Brake/gas, cruise disengagement, cancel/override/
resume, and manual cruise buttons inhibit planner-connected ICBM. The existing
Nissan controller and Panda still enforce PPP and cancellation constraints.

Cruise button timing helpers were moved to a lightweight shared module so
controller tests do not require native Params libraries. ICBM timer dictionaries
are now private to each instance.

## Validation (2026-11-01)

Offline replay of route 00000052--e778adf53f segment 20 first requests decrease
near 2026-10-31 21:48:24.89 JST: recorded speed 40.7 km/h, requested acceleration
-0.301 m/s², existing target 45 km/h, new rounded target 37 km/h. The old path
requests no button at that instant. This precedes the recorded brake signal
by about 3.6 seconds. No CAN is sent by replay.

Segments 10, 20 and 21 were replayed independently. No decrease requests were
observed while the recorded brake was pressed or cruise was disabled. Segment
10 also exercises the existing curve-target reduction. Replay uses recorded
OEM speeds/setpoints and cannot simulate how the vehicle responds to new
buttons. Freshness checks use log timestamps; offline replay does not simulate
SubMaster frequency health or process scheduling.

Tests cover target selection, existing curve bounds, units/minima, driver
priority, missing/stale/invalid plans, restoration, platform gating, and source
freshness. Nissan parking/PPP tests are included. Device build/runtime and
actual deceleration response remain unverified. This is not emergency braking.

```sh
PYTHONPATH=.:opendbc_repo:msgq_repo .venv/bin/python -m pytest -q \
  openpilot/sunnypilot/selfdrive/car/tests/test_planner_icbm.py \
  openpilot/sunnypilot/selfdrive/selfdrived/tests/test_parking.py \
  opendbc_repo/opendbc/sunnypilot/car/nissan/tests/test_parking.py

PYTHONPATH=.:opendbc_repo .venv/bin/python \
  tools/scripts/replay_planner_icbm.py /path/to/rlog.zst > decisions.jsonl
```

The replay tool processes one segment, assumes metric display, starts with an
empty ICBM state, and compares planner-connected and existing target selection.
Raw logs are kept outside the repository.
