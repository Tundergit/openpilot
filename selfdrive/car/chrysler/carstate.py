from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["SHIFTER_ASSM"]['SHIFTER_POSITION']

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

   # self.frame = int(cp.vl["EPS_STATUS"]['COUNTER']) THIS IS THE COUNTER I AM UNSURE OF (SEE BELOW)

    ret.doorOpen = any([cp.vl["DOORS"]['DOOR_OPEN_LF'],
                        cp.vl["DOORS"]['DOOR_OPEN_RF'],
                        cp.vl["DOORS"]['DOOR_OPEN_LR'],
                        cp.vl["DOORS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = cp.vl["DRIVER_SEATBELT_STATUS"]['DRIVER_SEATBELT_STATUS'] == 1

    ret.brakePressed = cp.vl["BRAKE_PEDAL"]['BRK_PEDAL'] > 1  # human-only
    ret.brake = 0
   # ret.brakeLights = ret.brakePressed I DON'T BELIEVE WE HAVE BRAKE LIGHT SIGNAL
    ret.gas = cp.vl["GAS_PEDAL"]['THROTTLE_POSITION']
    ret.gasPressed = ret.gas > 1e-5 

    ret.espDisabled = (cp.vl["CENTER_STACK"]['TRAC_OFF'] == 1)

    ret.wheelSpeeds.fl = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_LF']
    ret.wheelSpeeds.rr = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RR']
    ret.wheelSpeeds.rl = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_LR']
    ret.wheelSpeeds.fr = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RF']
  #  ret.vEgoRaw = (cp.vl['SPEED_1']['SPEED_LEFT'] + cp.vl['SPEED_1']['SPEED_RIGHT']) / 2. WE DON'T GET THESE SPEEDS, BUT COULD USE WHEEL SPEEDS HERE
  #  ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001

    ret.leftBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2
    ret.steeringAngle = cp.vl["EPS_1"]['STEER_ANGLE']
    ret.steeringRate = cp.vl["EPS_2"]['STEERING_RATE_DRIVER']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl['SHIFTER_ASSM']['SHIFTER_POSITION'], None))

    ret.cruiseState.enabled = cp.vl["FORWARD_CAMERA_ACC"]['ACC_STATUS'] == 3  # ACC is green.
    ret.cruiseState.available = ret.cruiseState.enabled  # FIXME: for now same as enabled
   # ret.cruiseState.speed = cp.vl["DASHBOARD"]['ACC_SPEED_CONFIG_KPH'] * CV.KPH_TO_MS
    # CRUISE_STATE is a three bit msg, 0 is off, 1 and 2 are Non-ACC mode, 3 and 4 are ACC mode, find if there are other states too
   # ret.cruiseState.nonAdaptive = cp.vl["DASHBOARD"]['CRUISE_STATE'] in [1, 2]

    ret.steeringTorque = cp.vl["EPS_2"]["TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["EPS_1"]["TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    steer_state = cp.vl["FORWARD_CAMERA_LKAS"]["LKAS_CONTROL_BIT"]
    ret.steerError = steer_state == 4 or (steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)

    ret.genericToggle = bool(cp.vl["STEERING_LEVERS"]['HIGH_BEAM_FLASH'])

   # self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]['COUNTER']
   # self.lkas_car_model = cp_cam.vl["LKAS_HUD"]['CAR_MODEL']
   # self.lkas_status_ok = cp_cam.vl["LKAS_HEARTBIT"]['LKAS_STATUS_OK']

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("SHIFTER_POSITION", "SHIFTER_ASSM", 0),
      ("DOOR_OPEN_LF", "DOORS", 0),
      ("DOOR_OPEN_RF", "DOORS", 0),
      ("DOOR_OPEN_LR", "DOORS", 0),
      ("DOOR_OPEN_RR", "DOORS", 0),
      ("BRK_PEDAL", "BRAKE_PEDAL", 0),
      ("THROTTLE_POSITION", "GAS_PEDAL", 0),
      #("SPEED_LEFT", "SPEED_1", 0), NOT NEEDED AT THIS POINT?
      #("SPEED_RIGHT", "SPEED_1", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_LR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RF", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_LF", "WHEEL_SPEEDS", 0),
      ("STEER_ANGLE", "EPS_1", 0),
      ("STEERING_RATE_DRIVER", "EPS_2", 0),
      ("TURN_SIGNALS", "STEERING_LEVERS", 0),
      ("ACC_STATUS", "FORWARD_CAMERA_ACC", 0),
      ("HIGH_BEAM_FLASH", "STEERING_LEVERS", 0),
      ("ACC_SET_SPEED", "FORWARD_CAMERA_CLUSTER", 0),
      ("ACC_STATUS", "FORWARD_CAMERA_ACC", 0),
      ("TORQUE_DRIVER", "EPS_2", 0),
      ("TORQUE_MOTOR", "EPS_1", 0),
      ("LKAS_CONTROL_BIT", "FORWARD_CAMERA_LKAS", 1),
      #("COUNTER", "EPS_STATUS", -1),    THERE ARE MANY COUNTERS NOT SURE WHICH ONE I NEED HERE, MAYBE NEED TO SEE WHERE ITS USED TO KNOW WHAT IT IS
      ("TRAC_OFF", "CENTER_STACK", 0),
      ("DRIVER_SEATBELT_STATUS", "DRIVER_SEATBELT_STATUS", 0),
    ]

    checks = [
      # sig_address, frequency
      ("BRAKE_PEDAL", 50),
      ("EPS_1", 100),
      ("EPS_2", 100),
      ("WHEEL_SPEEDS", 50),
      ("FORWARD_CAMERA_ACC", 50),
      ("FORWARD_CAMERA_CLUSTER", 50),
      ("FORWARD_CAMERA_LKAS", 50),
      ("SHIFTER_ASSM", 50),
      ("GAS_PEDAL", 50),
     # ("DASHBOARD", 15),
      ("STEERING_LEVERS", 10),
      ("DRIVER_SEATBELT_STATUS", 1),
      ("DOORS", 1),
      ("CENTER_STACK", 20),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("COUNTER", "LKAS_COMMAND", -1),
      ("CAR_MODEL", "LKAS_HUD", -1),
      ("LKAS_STATUS_OK", "LKAS_HEARTBIT", -1)
    ]
    checks = [
      ("LKAS_COMMAND", 100),
      ("LKAS_HEARTBIT", 10),
      ("LKAS_HUD", 4),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
