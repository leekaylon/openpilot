from cereal import car
from common.numpy_fast import mean
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, TSS2_CAR, EPS_SCALE
from selfdrive.swaglog import cloudlog
from common.realtime import DT_CTRL, sec_since_boot

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    # All TSS2 car have the accurate sensor
    self.accurate_steer_angle_seen = CP.carFingerprint in TSS2_CAR

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
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

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      ret.gasPressed = ret.gas > 15
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

    ret.standstill = ret.vEgoRaw < 0.001

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
          ret.usingZss = True
      else:
        ret.steeringAngleDeg = self.stock_steer_value
        self.steertype = 1
        ret.usingZss = False
    elif self.accurate_steer_angle_seen:
      ret.steeringAngleDeg = self.torque_steer_value
      self.steertype = 2
      ret.usingZss = False
    else:
      ret.steeringAngleDeg = self.stock_steer_value
      self.steertype = 1
      ret.usingZss = False

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
    ret.steerWarning = cp.vl["EPS_STATUS"]["LKA_STATE"] not in (1, 5)

    if self.CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
    else:
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS

    if self.CP.carFingerprint in TSS2_CAR:
      self.acc_type = cp_cam.vl["ACC_CONTROL"]["ACC_TYPE"]

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in (CAR.LEXUS_IS, CAR.LEXUS_RC)) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]["LKA_STATE"]

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

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
      ("TC_DISABLED", "ESP_CONTROL"),
      ("BRAKE_HOLD_ACTIVE", "ESP_CONTROL"),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR"),
      ("STEER_RATE", "STEER_ANGLE_SENSOR"),
      ("CRUISE_ACTIVE", "PCM_CRUISE"),
      ("CRUISE_STATE", "PCM_CRUISE"),
      ("GAS_RELEASED", "PCM_CRUISE"),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR"),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR"),
      ("TURN_SIGNALS", "BLINKERS_STATE"),
      ("LKA_STATE", "EPS_STATUS"),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK"),
    ]

    checks = [
      ("GEAR_PACKET", 1),
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    if CP.flags & ToyotaFlags.HYBRID:
      signals.append(("GAS_PEDAL", "GAS_PEDAL_HYBRID"))
      checks.append(("GAS_PEDAL_HYBRID", 33))
    else:
      signals.append(("GAS_PEDAL", "GAS_PEDAL"))
      checks.append(("GAS_PEDAL", 33))

    if CP.carFingerprint in (CAR.LEXUS_IS, CAR.LEXUS_RC):
      signals.append(("MAIN_ON", "DSU_CRUISE"))
      signals.append(("SET_SPEED", "DSU_CRUISE"))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2"))
      signals.append(("SET_SPEED", "PCM_CRUISE_2"))
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

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("FORCE", "PRE_COLLISION"),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION"),
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42),
      ("PRE_COLLISION", 0), # TODO: figure out why freq is inconsistent
    ]

    if CP.carFingerprint in TSS2_CAR:
      signals.append(("ACC_TYPE", "ACC_CONTROL"))
      checks.append(("ACC_CONTROL", 33))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
