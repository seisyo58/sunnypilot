#!/usr/bin/env python3
"""Replay one local rlog segment through ICBM without publishing or sending CAN.

Usage: PYTHONPATH=.:opendbc_repo python tools/scripts/replay_planner_icbm.py rlog.zst
Emits changed decisions and 1 Hz samples as JSONL. OEM responses are recorded,
not simulated: this cannot predict the resulting vehicle speed or stopping.
"""
import argparse
import bz2
import json
from pathlib import Path

import zstandard

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement, SendButtonState, planner_data_valid,
)


BUTTON_NAMES = {SendButtonState.none: "none", SendButtonState.increase: "increase", SendButtonState.decrease: "decrease"}


class ReplayState(dict):
  def __init__(self):
    super().__init__()
    self.logMonoTime = {}
    self.valid = {}

  def all_checks(self, service_list):
    return all(self.valid.get(s, False) for s in service_list)


def replay(path):
  data = Path(path).read_bytes()
  if path.endswith('.zst'):
    with zstandard.ZstdDecompressor().stream_reader(data) as stream:
      data = stream.read()
  elif path.endswith('.bz2'):
    data = bz2.decompress(data)
  messages = sorted(log.Event.read_multiple_bytes(data), key=lambda m: m.logMonoTime)
  cp = next(m.carParams for m in messages if m.which() == 'carParams')
  cp_sp = next(m.carParamsSP for m in messages if m.which() == 'carParamsSP')
  # Match the selfdrived call cadence (carState, usually 100 Hz).
  current = IntelligentCruiseButtonManagement(cp, cp_sp)
  baseline = IntelligentCruiseButtonManagement(cp, cp_sp)
  baseline.use_planner = False
  sm = ReplayState()
  last_decision, last_log = None, -float('inf')
  for msg in messages:
    kind = msg.which()
    now = msg.logMonoTime * 1e-9
    if kind in ('carState', 'carControl', 'longitudinalPlan', 'longitudinalPlanSP'):
      sm[kind] = getattr(msg, kind)
      sm.logMonoTime[kind], sm.valid[kind] = msg.logMonoTime, msg.valid
    if kind != 'carState' or len(sm) != 4:
      continue
    cs, cc, sp, plan = (sm[k] for k in ('carState', 'carControl', 'longitudinalPlanSP', 'longitudinalPlan'))
    for controller in (current, baseline):
      controller.run(cs, cc, sp, True, plan, plan_valid=planner_data_valid(sm, now))
    decision = (current.v_target, BUTTON_NAMES[current.cruise_button], BUTTON_NAMES[baseline.cruise_button])
    if decision != last_decision or now - last_log >= 1.:
      yield {"mono": now, "ego_kph": cs.vEgo * 3.6, "plan_a": plan.aTarget,
                 "plan_source": str(plan.longitudinalPlanSource), "sp_target_kph": sp.vTarget * 3.6,
                 "target_kph": current.v_target, "button": BUTTON_NAMES[current.cruise_button],
                 "baseline_target_kph": baseline.v_target, "baseline_button": BUTTON_NAMES[baseline.cruise_button],
                 "brake": cs.brakePressed, "acc_enabled": cs.cruiseState.enabled,
                 "plan_valid": planner_data_valid(sm, now), "shadow": True}
      last_decision, last_log = decision, now


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('log')
  args = parser.parse_args()
  for row in replay(args.log):
    print(json.dumps(row, allow_nan=False))
