import copy

from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import mean
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR
from system.swaglog import cloudlog
from common.realtime import DT_CTRL, sec_since_boot


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    # All TSS2 car have the accurate sensor
    self.accurate_steer_angle_seen = CP.carFingerprint in TSS2_CAR
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset_torque = CP.carFingerprint not in TSS2_CAR #offset only if needed
    self.needs_angle_offset_zss = True #ZSS always needs offset
    self.angle_offset_zss = 0.
    self.angle_offset_torque = 0.
    self.stock_steer_value = 0.
    self.zorro_steer_value = 0.
    self.torque_steer_value = 0.
    self.cruise_active_previous = False
    self.cruise_active = False
    self.out_of_tolerance_counter = 0
    self.steertype = 0 #for debug purposes. 0-undefined, 1-stock, 2-torque, 3-zorro
    self.count = 0
    self.low_speed_lockout = False
    self.acc_type = 1
    self.lkas_hud = {}

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = ret.gas > 805
    else:
      # TODO: find a new, common signal
      msg = "GAS_PEDAL_HYBRID" if (self.CP.flags & ToyotaFlags.HYBRID) else "GAS_PEDAL"
      ret.gas = cp.vl[msg]["GAS_PEDAL"]
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = ret.vEgoRaw == 0

    self.stock_steer_value = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
    if abs(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]) > 1e-3:
      self.accurate_steer_angle_seen = True

    if bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"]) and not self.cruise_active_previous:
      self.needs_angle_offset_zss = True # cruise was just activated, so allow offset to be recomputed
      self.out_of_tolerance_counter = 0 # Allow ZSS re-use after disengage and re-engage
    self.cruise_active_previous = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])

    #compute offset for torque steer
    if self.accurate_steer_angle_seen:
      if self.needs_angle_offset_torque:
        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
        if (abs(angle_wheel) > 1e-3 and abs(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]) > 1e-3):
          self.needs_angle_offset_torque = False
          self.angle_offset_torque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"] - angle_wheel
      self.torque_steer_value = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"] - self.angle_offset_torque

    if self.CP.hasZss:
      #compute offset for zorro steer
      if self.needs_angle_offset_zss:
        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
        if (abs(angle_wheel) > 1e-3 and abs(cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"]) > 1e-3):
          self.needs_angle_offset_zss = False
          self.angle_offset_zss = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"] - angle_wheel
      self.zorro_steer_value = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"] - self.angle_offset_zss

      #default to stock if 1) too many instances of steering being out of tolerance; something is not right
      #                    2) zorro steer offset has not been computed
      if self.out_of_tolerance_counter < 10 and not self.needs_angle_offset_zss:
        #check if zorro steer is out of tolerance
        if abs(self.stock_steer_value - self.zorro_steer_value) > 4.0:
          ret.steeringAngleDeg = self.stock_steer_value
          self.steertype = 1
          if self.cruise_active:
            self.out_of_tolerance_counter = self.out_of_tolerance_counter + 1 #should not get here too often with cruise active
        else:
          ret.steeringAngleDeg= self.zorro_steer_value
          self.steertype = 3
      else:
        ret.steeringAngleDeg = self.stock_steer_value
        self.steertype = 1
    elif self.accurate_steer_angle_seen:
      ret.steeringAngleDeg = self.torque_steer_value
      self.steertype = 2
    else:
      ret.steeringAngleDeg = self.stock_steer_value
      self.steertype = 1

    if (self.count % int(1.0 / DT_CTRL)) == 0:
      cloudlog.info("*** Zorro       *** %s" % self.zorro_steer_value)
      cloudlog.info("*** Torque      *** %s" % self.torque_steer_value)
      cloudlog.info("*** Stock       *** %s" % self.stock_steer_value)
      cloudlog.info("*** Num of OTs  *** %s" % self.out_of_tolerance_counter)
      cloudlog.info("*** OP is Using *** %s" % ret.steeringAngleDeg)
      steertypeText = "Undefined" #should never happen
      if self.steertype == 1:
        steertypeText = "Stock"
      elif self.steertype == 2:
        steertypeText = "Torque"
      elif self.steertype == 3:
        steertypeText = "Zorro"
      cloudlog.info("====================================")
      cloudlog.info("******* Using Steer Type: ******* %s" % steertypeText)
      cloudlog.info("====================================")

    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    # steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
    # lka msg drop out: goes to 9 then 11 for a combined total of 2 seconds
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in (0, 9, 11, 21, 25)
    # 17 is a fault from a prolonged high torque delta between cmd and user
    # 3 is a fault from the lka command message not being received by the EPS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in (3, 17)

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    if ret.cruiseState.speed != 0:
      is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    cp_acc = cp_cam if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) else cp

    if self.CP.carFingerprint in (TSS2_CAR | RADAR_ACC_CAR):
      if not (self.CP.flags & ToyotaFlags.SMART_DSU.value):
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["ACC_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      # ignore standstill state in certain vehicles, since pcm allows to restart with just an acceleration request
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if not self.CP.enableDsu:
      ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    self.count = self.count + 1

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),
      ("GEAR", "GEAR_PACKET"),
      ("BRAKE_PRESSED", "BRAKE_MODULE"),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS"),
      ("DOOR_OPEN_FL", "BODY_CONTROL_STATE"),
      ("DOOR_OPEN_FR", "BODY_CONTROL_STATE"),
      ("DOOR_OPEN_RL", "BODY_CONTROL_STATE"),
      ("DOOR_OPEN_RR", "BODY_CONTROL_STATE"),
      ("SEATBELT_DRIVER_UNLATCHED", "BODY_CONTROL_STATE"),
      ("PARKING_BRAKE", "BODY_CONTROL_STATE"),
      ("UNITS", "BODY_CONTROL_STATE_2"),
      ("TC_DISABLED", "ESP_CONTROL"),
      ("BRAKE_HOLD_ACTIVE", "ESP_CONTROL"),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE", "STEER_ANGLE_SENSOR"),
      ("CRUISE_ACTIVE", "PCM_CRUISE"),
      ("CRUISE_STATE", "PCM_CRUISE"),
      ("GAS_RELEASED", "PCM_CRUISE"),
      ("UI_SET_SPEED", "PCM_CRUISE_SM"),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE_INITIALIZING", "STEER_TORQUE_SENSOR"),
      ("TURN_SIGNALS", "BLINKERS_STATE"),
      ("LKA_STATE", "EPS_STATUS"),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK"),
    ]

    checks = [
      ("GEAR_PACKET", 1),
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("BODY_CONTROL_STATE_2", 2),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("PCM_CRUISE_SM", 1),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    if CP.flags & ToyotaFlags.HYBRID:
      signals.append(("GAS_PEDAL", "GAS_PEDAL_HYBRID"))
      checks.append(("GAS_PEDAL_HYBRID", 33))
    else:
      signals.append(("GAS_PEDAL", "GAS_PEDAL"))
      checks.append(("GAS_PEDAL", 33))

    if CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      signals.append(("MAIN_ON", "DSU_CRUISE"))
      signals.append(("SET_SPEED", "DSU_CRUISE"))
      signals.append(("UI_SET_SPEED", "PCM_CRUISE_ALT"))
      checks.append(("DSU_CRUISE", 5))
      checks.append(("PCM_CRUISE_ALT", 1))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2"))
      signals.append(("SET_SPEED", "PCM_CRUISE_2"))
      signals.append(("ACC_FAULTED", "PCM_CRUISE_2"))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2"))
      checks.append(("PCM_CRUISE_2", 33))

    if CP.carFingerprint == CAR.PRIUS:
      signals += [("STATE", "AUTOPARK_STATUS", 0)]
      checks += [("AUTOPARK_STATUS", 0)]
    if CP.hasZss:
      signals += [("ZORRO_STEER", "SECONDARY_STEER_ANGLE", 0)]
      checks += [("SECONDARY_STEER_ANGLE", 0)]

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR"))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR"))
      checks.append(("GAS_SENSOR", 50))

    if CP.enableBsm:
      signals += [
        ("L_ADJACENT", "BSM"),
        ("L_APPROACHING", "BSM"),
        ("R_ADJACENT", "BSM"),
        ("R_APPROACHING", "BSM"),
      ]
      checks.append(("BSM", 1))

    if CP.carFingerprint in RADAR_ACC_CAR:
      if not CP.flags & ToyotaFlags.SMART_DSU.value:
        signals += [
          ("ACC_TYPE", "ACC_CONTROL"),
        ]
        checks += [
          ("ACC_CONTROL", 33),
        ]
      signals += [
        ("FCW", "ACC_HUD"),
      ]
      checks += [
        ("ACC_HUD", 1),
      ]

    if CP.carFingerprint not in (TSS2_CAR - RADAR_ACC_CAR) and not CP.enableDsu:
      signals += [
        ("FORCE", "PRE_COLLISION"),
        ("PRECOLLISION_ACTIVE", "PRE_COLLISION"),
      ]
      checks += [
        ("PRE_COLLISION", 33),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []

    if CP.carFingerprint != CAR.PRIUS_V:
      signals += [
        ("LANE_SWAY_FLD", "LKAS_HUD"),
        ("LANE_SWAY_BUZZER", "LKAS_HUD"),
        ("LANE_SWAY_WARNING", "LKAS_HUD"),
        ("LANE_SWAY_SENSITIVITY", "LKAS_HUD"),
        ("LANE_SWAY_TOGGLE", "LKAS_HUD"),
      ]
      checks += [
        ("LKAS_HUD", 1),
      ]

    if CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      signals += [
        ("PRECOLLISION_ACTIVE", "PRE_COLLISION"),
        ("FORCE", "PRE_COLLISION"),
        ("ACC_TYPE", "ACC_CONTROL"),
        ("FCW", "ACC_HUD"),
      ]
      checks += [
        ("PRE_COLLISION", 33),
        ("ACC_CONTROL", 33),
        ("ACC_HUD", 1),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
