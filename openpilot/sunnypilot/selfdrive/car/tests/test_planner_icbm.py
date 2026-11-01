import math
from types import SimpleNamespace as NS

import pytest

from opendbc.car import structs
from opendbc.car.nissan.values import CAR
from openpilot.cereal import custom, log
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement, SendButtonState, planner_data_valid,
)


def setup(car=CAR.NISSAN_LEAF_IC, speed=60., target=80.):
  cp = structs.CarParams(carFingerprint=car)
  controller = IntelligentCruiseButtonManagement(cp, custom.CarParamsSP(pcmCruiseSpeed=False))
  cs = structs.CarState(vEgo=speed/3.6, cruiseState={'enabled': True, 'speedCluster': target/3.6})
  cc = structs.CarControl(enabled=True)
  sp = custom.LongitudinalPlanSP(vTarget=target/3.6)
  plan = log.LongitudinalPlan.new_message(aTarget=-1.)
  return controller, cs, cc, sp, plan


def tick(c, cs, cc, sp, plan, valid=True, metric=True, frames=60):
  buttons = []
  for _ in range(frames):
    c.run(cs, cc, sp, metric, plan, plan_valid=valid)
    buttons.append(c.cruise_button)
  return buttons


@pytest.mark.parametrize('car', [CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC])
def test_deceleration_reaches_button_sender(car):
  c, cs, cc, sp, plan = setup(car=car)
  assert SendButtonState.decrease in tick(c, cs, cc, sp, plan)
  assert 78 <= c.v_target < 80  # bounded adjustment from acknowledged OEM setpoint


def test_curve_and_speed_limit_target_remains_upper_bound():
  c, cs, cc, sp, plan = setup(target=40.)
  tick(c, cs, cc, sp, plan)
  assert c.v_target <= 40


@pytest.mark.parametrize('accel', [-.29, 0., 1.])
def test_no_planner_cap_without_meaningful_deceleration(accel):
  c, cs, cc, sp, plan = setup()
  plan.aTarget = accel
  tick(c, cs, cc, sp, plan)
  assert c.v_target == 80


@pytest.mark.parametrize('metric,expected', [(True, 30), (False, 20)])
def test_oem_minimum(metric, expected):
  c, cs, cc, sp, plan = setup(speed=35., target=30. if metric else 20 * 1.609344)
  plan.aTarget = -3.
  tick(c, cs, cc, sp, plan, metric=metric)
  assert c.v_target == expected


def test_other_nissan_unchanged():
  c, cs, cc, sp, plan = setup(car=CAR.NISSAN_XTRAIL)
  tick(c, cs, cc, sp, plan, valid=False)
  assert c.v_target == 80


@pytest.mark.parametrize('fault', ['stale', 'missing', 'nan', 'brake', 'gas', 'cancel', 'override', 'resume', 'disabled', 'cruise_off'])
def test_faults_and_driver_actions_stop_button_output(fault):
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan)
  assert c.cruise_button == SendButtonState.decrease
  valid = fault != 'stale'
  if fault == 'missing':
    plan = None
  elif fault == 'nan':
    plan.aTarget = math.nan
  elif fault in ('brake', 'gas'):
    setattr(cs, fault + 'Pressed', True)
  elif fault in ('cancel', 'override', 'resume'):
    setattr(cc.cruiseControl, fault, True)
  elif fault == 'disabled':
    cc.enabled = False
  elif fault == 'cruise_off':
    cs.cruiseState.enabled = False
  assert all(b == SendButtonState.none for b in tick(c, cs, cc, sp, plan, valid=valid))


def test_manual_button_has_priority():
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan)
  cs.buttonEvents = [structs.CarState.ButtonEvent(type='decelCruise', pressed=True)]
  assert tick(c, cs, cc, sp, plan, frames=1) == [SendButtonState.none]


def test_recovery_uses_existing_acceleration_and_user_target():
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan)
  cs.cruiseState.speedCluster = 49/3.6
  plan.aTarget = 0.
  assert SendButtonState.increase in tick(c, cs, cc, sp, plan)
  assert c.v_target <= 80


def test_timer_state_is_not_shared():
  c, *_ = setup()
  other, *_ = setup()
  c.cruise_button_timers[next(iter(c.cruise_button_timers))] = 10
  assert not any(other.cruise_button_timers.values())


def test_missing_plan_does_not_restore_higher_target():
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan)
  cs.cruiseState.speedCluster = 49/3.6
  assert all(b == SendButtonState.none for b in tick(c, cs, cc, sp, plan, valid=False))
  assert c.v_target < 80


def test_no_decrease_below_oem_minimum():
  c, cs, cc, sp, plan = setup(speed=20., target=30.)
  assert all(b == SendButtonState.none for b in tick(c, cs, cc, sp, plan))


def test_disabled_icbm_sends_nothing():
  c, cs, cc, sp, plan = setup()
  c.CP_SP.pcmCruiseSpeed = True
  assert all(b == SendButtonState.none for b in tick(c, cs, cc, sp, plan))


def test_existing_curve_reduction_without_ai_deceleration():
  c, cs, cc, sp, plan = setup()
  plan.aTarget = 0.
  sp.vTarget = 40/3.6
  assert SendButtonState.decrease in tick(c, cs, cc, sp, plan)
  assert c.v_target == 40


@pytest.mark.parametrize('fault', ['none', 'invalid', 'old_plan', 'old_sp', 'old_model', 'future'])
def test_source_freshness(fault):
  sm = type('SM', (dict,), {})(longitudinalPlan=NS(modelMonoTime=100_000_000_000))
  sm.logMonoTime = {'longitudinalPlan': 100_000_000_000, 'longitudinalPlanSP': 100_000_000_000}
  sm.all_checks = lambda **kw: fault != 'invalid'
  if fault == 'old_plan':
    sm.logMonoTime['longitudinalPlan'] -= 1_000_000_000
  elif fault == 'old_sp':
    sm.logMonoTime['longitudinalPlanSP'] -= 1_000_000_000
  elif fault == 'old_model':
    sm['longitudinalPlan'].modelMonoTime -= 1_000_000_000
  elif fault == 'future':
    sm.logMonoTime['longitudinalPlan'] += 1_000_000_000
  assert planner_data_valid(sm, 100.1) == (fault == 'none')


def test_sufficient_actual_deceleration_stops_lowering():
  c, cs, cc, sp, plan = setup()
  cs.aEgo = -1.2
  tick(c, cs, cc, sp, plan, frames=500)
  assert c.v_target >= 80
  assert c.cruise_button != SendButtonState.decrease


def test_falling_speed_does_not_drag_target_down():
  c, cs, cc, sp, plan = setup(target=50.)
  cs.aEgo = -1.
  for speed in [50., 45., 40., 35.]:
    cs.vEgo = speed / 3.6
    tick(c, cs, cc, sp, plan, frames=100)
    assert c.v_target == 50


def test_request_release_is_rate_limited():
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan, frames=500)
  previous = c.planner_cap
  plan.aTarget = 0.
  tick(c, cs, cc, sp, plan, frames=1)
  assert 0 < c.planner_cap - previous <= .005001


def test_pending_reduction_is_bounded_without_oem_response():
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan, frames=1000)
  assert c.planner_cap >= cs.cruiseState.speedCluster - 2/3.6 - 1e-6


def test_threshold_hysteresis_preserves_deceleration_mode():
  c, cs, cc, sp, plan = setup()
  tick(c, cs, cc, sp, plan)
  plan.aTarget = -.2
  tick(c, cs, cc, sp, plan)
  assert c.planner_decelerating
  plan.aTarget = -.09
  tick(c, cs, cc, sp, plan)
  assert not c.planner_decelerating
