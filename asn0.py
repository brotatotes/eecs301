#!/usr/bin/env python
import roslib
import rospy
import pickle
from fw_wrapper.srv import *

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
            
    print(current_pos, r, d)
    
    return setMotorTargetPositionCommand(motor_id, d)
    
def deg_to_adc(motor_id, deg):
    if motor_id in (5,8):
        return 3.36 * deg + 210
    elif motor_id in (6,7):
        return -3.36 * deg + 815
    elif motor_id in (1,2,3,4):
        return 3.36 * deg + 512
    else:
        raise Exception("Unrecognized Motor ID.")

def go_to_default():
    go_to_state("default")
    
def go_to_state(statename):
    states = pickle.load( open("states.p", "rb"))
    state = states[statename]
    print("Attempting", statename)
    setMotorTargetPositionsSync(len(state), range(1,9), state)
    print("Completed", statename)
    
    
def releaseMotors():
    for i in range(1,9):
        setMotorMode(i,1)
        
def engageMotors():
    for i in range(1,9):
        setMotorMode(i,0)
        
def captureState(name):
    states = pickle.load( open("states.p", "rb"))
    states[name] = [getMotorPositionCommand(i) for i in range(1,9)]
    pickle.dump(states, open("states.p", "wb"))
    
def doBehavior1():
    behaviors = ["default", "crouch", "lean", "wiggle1", "wiggle2", "wiggle1", "wiggle2", "wiggle1", "wiggle2", "halfup", "default"]
    doBehavior(behaviors)
    
def turn(i, direction="clockwise"):
    if direction == "clockwise":
        behaviors = ["lift1", "swing1", "lift2"]
    else:
        behaviors = ["lift3", "swing3", "lift4"]
    doBehavior([behaviors[i%len(behaviors)]])
        
def doBehavior(behaviors):
    for b in behaviors:
        go_to_state(b)
        
# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")
    
    states = pickle.load( open("states.p", "rb"))
    print(states)
    
    
    capturing = raw_input("Capture mode? (y/n): ")
    capturing = True if capturing == 'y' else False
    
    
    
    if capturing:
        releaseMotors()
        s = raw_input("What is the name of this state? ")
        captureState(s)
        print(s, "saved")
        engageMotors()
    else: # testing area
        pass


    
                    
    # IR: 2
    IR = 2
    # DMS: 1
    DMS = 1
    
    go_to_default()

    # control loop running at X Hz
    r = rospy.Rate(1000) # 1000hz
    
    irs = [0] * 5
    threshold = 50
    step = 0
    objectSeen = False
    direction = "clockwise"
    while not rospy.is_shutdown():
        
        # behavior1
        irs[:len(irs)-1] = irs[1:]
        irs[-1] = getSensorValue(IR)
        print(irs, map(lambda x: True if x > threshold else False, irs))
        if all(map(lambda x: True if x > threshold else False, irs)):
            doBehavior1()

        else:
            if getSensorValue(DMS) > 1000:
                direction = "clockwise" if direction != "clockwise" else "counterclockwise"       
        
            step += 1
            turn(step, direction)
            
        # sleep to enforce loop rate
        r.sleep()
        

        
    
        

