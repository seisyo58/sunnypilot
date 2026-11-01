"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from __future__ import annotations

import math

from openpilot.cereal import custom, log
from opendbc.car.structs import car
from opendbc.car import DT_CTRL, structs, apply_hysteresis
from opendbc.car.nissan.values import CAR as NISSAN
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_buttons import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
PLANNER_LOOKAHEAD = 3.0  # s; convert requested acceleration to an OEM set-speed cap
PLANNER_DECEL_THRESHOLD = -0.3  # m/s²; ignore very small deceleration requests
PLAN_MAX_AGE = 0.25  # s


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


def planner_data_valid(sm, now: float) -> bool:
  services = ('longitudinalPlan', 'longitudinalPlanSP')
  stamps = [sm.logMonoTime[s] * 1e-9 for s in services]
  stamps.append(sm['longitudinalPlan'].modelMonoTime * 1e-9)
  return sm.all_checks(service_list=list(services)) and all(-1e-6 <= now - stamp <= PLAN_MAX_AGE for stamp in stamps)


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER.copy()
    self.use_planner = CP.carFingerprint in (NISSAN.NISSAN_LEAF, NISSAN.NISSAN_LEAF_IC) and not CP.openpilotLongitudinalControl

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP,
                          plan: log.LongitudinalPlan | None = None) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    target = LP_SP.vTarget
    if self.use_planner and plan is not None and plan.aTarget <= PLANNER_DECEL_THRESHOLD:
      minimum = get_minimum_set_speed(self.is_metric) * ms_conv
      planner_target = max(minimum, CS.vEgo + plan.aTarget * PLANNER_LOOKAHEAD)
      target = min(target, planner_target)
    self.v_target_ms_last = apply_hysteresis(target, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool,
          plan: log.LongitudinalPlan | None = None, *, plan_valid: bool = False) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_readiness(CS, CC)
    if self.use_planner:
      # A missing/stale plan must not restore the higher cruise target or send RES.
      self.is_ready &= (plan_valid and plan is not None and math.isfinite(plan.aTarget) and
                        math.isfinite(LP_SP.vTarget) and math.isfinite(CS.vEgo) and
                        not CS.brakePressed and not CS.gasPressed and CS.cruiseState.enabled)
    if not self.use_planner or self.is_ready:
      self.update_calculations(CS, LP_SP, plan)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
