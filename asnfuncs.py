#!/usr/bin/env python
import roslib, rospy, pickle, sys, math, time
from fw_wrapper.srv import *
from map import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to SetMotorTargetPositionsSync
def setMotorTargetPositionsSync(n, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPositionsSync', 0, 0, n, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def setWheelSpeedSync(n, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', 0, 0, n, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#################################### Motor Handling ####################################

def moveMotor(motor_id, deg):
    # thighs: top down counter-clockwise
    # legs: down, up
    motor_ranges = {1:(160,720), 2:(280,850), 3:(280,850), 4:(160,720),
                    5:(160,820), 6:(860,200), 7:(860,200), 8:(160,820)}

    if not motor_id in range(1,9):
        raise Exception("Unrecognized Motor ID.")

    adc = deg_to_adc(motor_id, deg)

    r = motor_ranges[motor_id]

    current_pos = getMotorPositionCommand(motor_id)

    if adc >= 0:
        if r[0] < r[1]:
            d = min(adc, r[1])
        else:
            d = max(adc, r[1])
    else:
        if r[0] < r[1]:
            d = max(adc, r[0])
        else:
            d = min(adc, r[0])

    print("Attempting to move from", current_pos, "to", d, "within", r)

    return setMotorTargetPositionCommand(motor_id, d)

    print("Motor move completed.")

def deg_to_adc(motor_id, deg):
    if motor_id in (5,8):
        return 3.36 * deg + 210
    elif motor_id in (6,7):
        return -3.36 * deg + 815
    elif motor_id in (1,2,3,4):
        return 3.36 * deg + 512
    else:
        raise Exception("Unrecognized Motor ID.")

def adc_to_deg(motor_id, adc):
    if motor_id in (5,8):
        return (adc - 210) / 3.36
    elif motor_id in (6,7):
        return (adc - 815) / -3.36
    elif motor_id in (1,2,3,4):
        return (adc - 512) / 3.36
    else:
        raise Exception("Unrecognized Motor ID.")

def deg_to_adc_l(l):
    if len(l) != 8:
        raise Exception("Need 8 entries")
    return [deg_to_adc(i+1, d) for i,d in enumerate(l)]

def adc_to_deg_l(l):
    if len(l) != 8:
        raise Exception("Need 8 entries")
    return [adc_to_deg(i+1, d) for i,d in enumerate(l)]

def releaseMotors():
    for i in range(1,9) + range(11,15):
        setMotorMode(i,1)

def engageMotors():
    for i in range(1,9) + range(11,15):
        setMotorMode(i,0)

#################################### Wheel Handling #####################################

def drive(direction, state):
    # we don't know what 0 direction means yet
    d = direction.lower()
    if d == "n":
        if state == 'X':
            xToY()
            state = 'Y'
        speeds = [1023, 2047, 1023, 2047]
    elif d == "s":
        if state == 'X':
            xToY()
            state = 'Y'
        speeds = [2047, 1023, 2047, 1023]
    elif d == "w":
        if state == 'Y':
            yToX()
            state = 'X'
        speeds = [1023, 1023, 2047, 2047]
    elif d == "e":
        if state == 'Y':
            yToX()
            state = 'X'
        speeds = [2047, 2047, 1023, 1023]

    for i in range(11,15):
        setMotorMode(i, 1)

    setWheelSpeedSync(4, range(11,15), speeds)

    time.sleep(1.8)

    stopDrive()

    return state

def wheelTurn():
    speeds = [1023] * 4

    for i in range(11,15):
        setMotorMode(i, 1)

    setWheelSpeedSync(4, range(11,15), speeds)
    time.sleep(2)
    stopDrive()

def xToY():

    # raw_input()
    moveMotor(7, 15)
    moveMotor(3, 90)
    moveMotor(7, 0)

    # raw_input()
    moveMotor(6, 15)
    moveMotor(2, 90)
    moveMotor(6, 0)

    # raw_input()
    moveMotor(8, 15)
    moveMotor(4, -90)
    # raw_input()

    moveMotor(5, 15)
    moveMotor(1, -90)
    moveMotor(5, 0)
    moveMotor(8, 0)



def yToX():
    moveMotor(6, 15)
    moveMotor(2, 0)
    moveMotor(6, 0)

    moveMotor(7, 15)
    moveMotor(3, 0)
    moveMotor(7, 0)

    moveMotor(5, 15)
    moveMotor(1, 0)
    moveMotor(5, 0)

    moveMotor(8, 15)
    moveMotor(4, 0)
    moveMotor(8, 0)

def switch(state = "x"):

    if state == "x":
        targets = [0] * 8
    else:
        targets = [-90, 90, 90, -90] + [0] * 4

    for i in range(1,9):
        # setMotorWheelSpeed(i, 1000)
        # moveMotor(i, targets[i-1])
        # time.sleep(1)
        targets[i-1] = deg_to_adc(i, targets[i-1])

    setMotorTargetPositionsSync(8, range(1,9), targets)



def stopDrive():
    setWheelSpeedSync(4, range(11,15), [0] * 4)

#################################### Sensor Handling #####################################
def get_sensor_consts(sensor):
    if sensor == 1:
        a, b = -7.4689, -0.1898
    elif sensor == 2:
        a, b = -7.5156, -0.1777
    else:
        a, b = -7.9882, -0.0505
    return a,b

def adc_to_cm(sensor, adc):
    """
    sensor == 1 is ir1
    sensor == 2 is ir2
    sensor == 0 is dms

    ir works up to ~ 25 cm
    dms works ~ 7cm to 60 cm
    """
    if adc <= 0:
        return float("inf")

    a, b = get_sensor_consts(sensor)

    return (math.log(adc) + a) / (b)

def cm_to_adc(sensor, cm):
    """
    sensor == 1 is ir1
    sensor == 2 is ir2
    sensor == 0 is dms

    ir works up to ~ 25 cm
    dms works ~ 7cm to 60 cm
    """
    a, b = get_sensor_consts(sensor)

    return math.exp(b * cm - a)

def viewSensors(sensors):
    """
    assumes sensors are the ports [IR1, IR2, DMS]
    """
    ir1, ir2, dms = sensors
    adc = map(getSensorValue, sensors)
    vals = [-1]*3
    for i,p in enumerate(sensors):
        vals[i] = adc_to_cm(i+1, adc[i])
    print "SENSOR DATA"
    print "IR1: %d adc\nIR2: %d adc\nDMS: %d adc" % (adc[0], adc[1], adc[2])
    print "IR1: %.2fcm\nIR2: %.2fcm\nDMS: %.2fcm\n" % (vals[0], vals[1], vals[2])

def detectObstacles(vals):
    left = tooclose(vals[0], 23)
    right = tooclose(vals[1], 23)
    front = tooclose(vals[2], 23)

    if not front:
        return "F"
    else:
        if left and right:
            return "180"
        elif left:
            return "R90"
        else:
            return "L90"

def tooclose(val, threshold):
    return all(map(lambda x: x <= threshold, val))

def toofar(val, threshold):
    return all(map(lambda x: x >= threshold, val))


#################################### State Management ####################################

def go_to_default():
    go_to_state("default")


def go_to_state(statename):
    if type(statename) is str:
        states = pickle.load( open("states.p", "rb"))
        state = states[statename]
    elif type(statename) is list:
        state = statename
    # print("Attempting", statename)
    setMotorTargetPositionsSync(len(state), range(1,9), state)
    # print("Completed", statename)

def captureState(name):
    states = pickle.load( open("states.p", "rb"))
    states[name] = [getMotorPositionCommand(i) for i in range(1,9)]
    pickle.dump(states, open("states.p", "wb"))

def doBehavior1():
    behaviors = ["default", "crouch", "lean", "wiggle1", "wiggle2", "wiggle1", "wiggle2", "wiggle1", "wiggle2", "halfup", "default"]
    doBehavior(behaviors)

def doBehavior(behaviors):
    for b in behaviors:
        go_to_state(b)

def viewStates():
    states = pickle.load(open("states.p", "rb"))
    for k in states.keys():
        degs = [-1] * 8
        for i in range(8):
            degs[i] = "%.2f" % adc_to_deg(i+1, states[k][i])
        print k, "\t\t", degs

#################################### Macro Actions ####################################

def fineleft():
    b = []
    for i in range(1,9):
        b.append("left"+str(i))
    b.append("default")
    doBehavior(b)

def fineright():
    b = []
    for i in range(1,9):
        b.append("right"+str(i))
    b.append("default")
    doBehavior(b)

def turn(i, direction="clockwise"):
    if direction == "clockwise":
        behaviors = ["lift1", "lift2"] # ["lift1", "swing1", "lift2"]
    else:
        behaviors = ["lift3", "lift4"] # ["lift3", "swing3", "lift4"]
    doBehavior([behaviors[i%len(behaviors)]])

def turn90(direction="clockwise"):
    if direction == "clockwise":
        b = ["lift2", "lift1"] * 4
        doBehavior(b)
        go_to_default()
    else:
        b = ["lift4", "lift3"] * 4 + ["lift4"]
        doBehavior(b)
        go_to_default()

def turn180(direction="clockwise"):
    if direction == "clockwise":
        b = ["lift2", "lift1"] * 9
        doBehavior(b)
        go_to_default()
    else:
        b = ["lift4", "lift3"] * 9 + ["lift4"]
        doBehavior(b)
        go_to_default()

def forward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["forward1"], states["forward2"]] * cycles
    doBehavior(b)

def rightforward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["forward2"], states["forward1"]] * cycles
    doBehavior(b)

def backward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["backward1"], states["backward2"]] * cycles
    doBehavior(b)

def strafeleft(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["strafeleft1"], states["strafeleft2"]] * cycles
    doBehavior(b)

def straferight(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["straferight1"], states["straferight2"]] * cycles
    doBehavior(b)
