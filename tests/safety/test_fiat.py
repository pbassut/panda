#!/usr/bin/env python3
import unittest
from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda


class TestFiatSafety(common.PandaCarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [
    [0x2FA, 1],
    [0x1F6, 0],
  ]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1F6]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_TORQUE = 261
  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000
  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_ERROR = 80

  DAS_BUS = 1

  def setUp(self):
    self.packer = CANPackerPanda("fca_fastback_limited_edition_2024_generated")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_FIAT, 0)
    self.safety.init_tests()

  def _button_msg(self, resume):
    values = {"CRUISE_BUTTON_PRESSED": 32 if resume else 128}
    return self.packer.make_can_msg_panda("DAS_1", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ENGAGED": 1 if enable else 0}
    return self.packer.make_can_msg_panda("DAS_2", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"VEHICLE_SPEED": speed }
    return self.packer.make_can_msg_panda("ABS_6", 0, values)

  def _user_gas_msg(self, gas):
    values = {"ACCEL_PEDAL_THRESHOLD": gas}
    return self.packer.make_can_msg_panda("ENGINE_1", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSURE": brake}
    return self.packer.make_can_msg_panda("ABS_6", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_panda("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_WATCH_STATUS": steer_req}
    return self.packer.make_can_msg_panda("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(resume=False)))

if __name__ == "__main__":
  unittest.main()
