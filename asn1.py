#!/usr/bin/env python
import roslib
import rospy
import pickle
from fw_wrapper.srv import *
import sys

# roscore
# rosrun fw_wrapper srv_wrapper
# rosrun eecs301_grp_c asnX.py

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

def go_to_default():
    go_to_state("default")
    
def go_to_state(statename):
    if type(statename) is str:
        states = pickle.load( open("states.p", "rb"))
        state = states[statename]
    elif type(statename) is list:
        state = statename
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
        behaviors = ["lift1", "lift2"] # ["lift1", "swing1", "lift2"]
    else:
        behaviors = ["lift3", "lift4"] # ["lift3", "swing3", "lift4"]
    doBehavior([behaviors[i%len(behaviors)]])

def turn90(direction="clockwise"):
    if direction == "clockwise":
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_default()
    else:
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_default()

def turn180(direction="clockwise"):
    if direction == "clockwise":
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_state("lift2")
        go_to_state("lift1")
        go_to_default()
    else:
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_state("lift3")
        go_to_state("lift4")
        go_to_default()
        
def forward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["forward1"], states["forward2"]] * cycles
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

def doBehavior(behaviors):
    for b in behaviors:
        go_to_state(b)
        
# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")
    
    states = pickle.load( open("states.p", "rb"))
    print("States loaded:", states.keys())
    
    print("Select one:")
    print("1. capture mode")
    print("2. asn0")
    print("3. asn1")
    print("4. test")
    selection = raw_input(">>> ")
    
    asn0 = asn1 = test = False
    
    if selection == "1":
        releaseMotors()
        s = raw_input("What is the name of this state? ")
        captureState(s)
        print(s, "saved")
        engageMotors()
        sys.exit()

    elif selection == "2":
        asn0 = True
        irs = [0] * 5
        threshold = 50
        step = 0
        objectSeen = False
        direction = "clockwise"

    elif selection == "3":
        asn1 = True
        # m1=[-90, 90, 0, 0, 0, 20, 20, 0]
        # m2=[0, 0, 90, -90, 20, 0, 0, 20]
        # for i in range(8):
        #     m1[i] = deg_to_adc(i+1, m1[i])
        #     m2[i] = deg_to_adc(i+1, m2[i])
        # print(m1,m2)
        go_to_default()

        # states["strafeleft1"] = m1
        # states["strafeleft2"] = m2
        # pickle.dump(states, open("states.p", "wb"))
        # print("dumped")

        raw_input("start? ")

        # forward(2)

        # strafeleft(2)

        # backward(2)

        # straferight(2)
        # s = "left"
        # for _ in range(6):
        #     for i in range(1,9):
        #         go_to_state(states[s+str(i)])
        #     go_to_default()

        # raw_input()

        turn90()

        raw_input("enter loop? ")

    else: # testing area
        for k in states.keys():
            degs = [-1] * 8
            for i in range(8):
                degs[i] = "%.2f" % adc_to_deg(i+1, states[k][i])
            print k, "\t\t", degs

        # raw_input("save new stuff? ")

        # convert = lambda l: [deg_to_adc(i+1, d) for i,d in enumerate(l)]

        # # states["swing1"] = convert([-45, 0, 0, -45, 0, 0, 0, 0])
        # # states["swing3"] = convert([0, 45, 45, 0, 0, 0, 0, 0])
        # # states["lift1"] = convert([-45, 45, 45, -45, 0, 20, 20, 0])
        # # states["lift2"] = convert([-90, 90, 90, -90, 20, 0, 0, 20])
        # # states["lift3"] = convert([-45, 45, 45, -45, 20, 0, 0, 20])
        # # states["lift4"] = convert([-90, 90, 90, -90, 0, 20, 20, 0])
        # # states["default"] = convert([-45, 45, 45, -45, 0, 0, 0, 0])

        # states["left1"] = convert([0, 45, 45, -45, 45, 0, 0, 0])
        # states["left2"] = convert([0, 45, 45, -45, 0, 0, 0, 0])
        # states["left3"] = convert([0, 45, 90, -45, 0, 0, 45, 0])
        # states["left4"] = convert([0, 45, 90, -45, 0, 0, 0, 0])
        # states["left5"] = convert([0, 45, 90, 0, 0, 0, 0, 45])
        # states["left6"] = convert([0, 45, 90, 0, 0, 0, 0, 0])
        # states["left7"] = convert([0, 90, 90, 0, 0, 45, 0, 0])
        # states["left8"] = convert([0, 90, 90, 0, 0, 0, 0, 0]) 
        # states["left9"] = convert([-90, 0, 0, -90, 0, 0, 0, 0])


        # pickle.dump(states, open("states.p", "wb"))

        raw_input("loop? ")
                  
    # IR: 2
    IR = 2
    # DMS: 1
    DMS = 1
    
    # go_to_default()

    # control loop running at X Hz
    r = rospy.Rate(1000) # 1000hz
    
    
    while not rospy.is_shutdown():
        
        if asn0:
            irs[:len(irs)-1] = irs[1:]
            irs[-1] = getSensorValue(IR)
            print(irs, map(lambda x: x > threshold, irs))
            if all(map(lambda x: x > threshold, irs)):
                doBehavior1()

            else:
                if getSensorValue(DMS) > 1000:
                    direction = "clockwise" if direction != "clockwise" else "counterclockwise"       
            
                step += 1
                turn(step, direction)

        elif asn1:
            go_to_default()
            #forward_walk 1

            # raw_input("next? ")

            #forward_walk 2

            # raw_input("next? ")

        elif test:
            pass


            
        # sleep to enforce loop rate
        r.sleep()
        

        
    
        

