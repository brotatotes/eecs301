#!/usr/bin/env python
from asnfuncs import *

# roscore
# rosrun fw_wrapper srv_wrapper
# rosrun eecs301_grp_c asnX.py
        
# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")
    
    # load states
    states = pickle.load( open("states.p", "rb"))
    print("States loaded:", states.keys())
    
    # select mode to run
    print("Select one:")
    print("1. capture mode")
    print("2. asn0")
    print("3. asn1")
    print("4. test")
    selection = raw_input(">>> ")
    
    asn0 = asn1 = test = False
    
    # setup for each  mode
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

        WALL = True
        REACT = False

        nvals = 5

        leftVals = [0] * nvals
        rightVals = [0] * nvals
        frontVals = [0] * nvals

        go_to_default()

        raw_input("enter loop? ")

    else: # testing area
        test = True

        # viewStates()
        # raw_input("save?")
        # states["right1"] = deg_to_adc_l([-90, 45, 45, -45, 20, 0, 0, 0])
        # states["right2"] = deg_to_adc_l([-90, 45, 45, -45, 0, 0, 0, 0])
        # states["right3"] = deg_to_adc_l([-90, 0, 45, -45, 0, 20, 0, 0])
        # states["right4"] = deg_to_adc_l([-90, 0, 45, -45, 0, 0, 0, 0])
        # states["right5"] = deg_to_adc_l([-90, 0, 45, -90, 0, 0, 0, 20])
        # states["right6"] = deg_to_adc_l([-90, 0, 45, -90, 0, 0, 0, 0])
        # states["right7"] = deg_to_adc_l([-90, 0, 0, -90, 0, 0, 20, 0])
        # states["right8"] = deg_to_adc_l([-90, 0, 0, -90, 0, 0, 0, 0])
        # pickle.dump(states, open("states.p", "wb"))

        raw_input("loop? ")
    
    # Sensor setup
    IR1 = 5 # left
    IR2 = 3 # right
    DMS = 1 # front
    SENSORS = (IR1, IR2, DMS)

    # IR 1 is port 1
    # IR 2 is port 2

    # control loop running at X Hz
    r = rospy.Rate(1) # 1000hz
    go_to_default()
    
    while not rospy.is_shutdown():
        
        # loop for each mode
        if asn0:
            irs[:len(irs)-1] = irs[1:]
            irs[-1] = getSensorValue(IR1)
            if all(map(lambda x: x > threshold, irs)):
                doBehavior1()

            else:
                if getSensorValue(DMS) > 1000:
                    direction = "clockwise" if direction != "clockwise" else "counterclockwise"       
            
                step += 1
                turn(step, direction)

        elif asn1:
            leftVals = [adc_to_cm(IR1, getSensorValue(IR1)) for _ in range(nvals)]
            rightVals = [adc_to_cm(IR2, getSensorValue(IR2)) for _ in range(nvals)]
            frontVals = [adc_to_cm(DMS, getSensorValue(DMS)) for _ in range(nvals)]



            if WALL:
                if within15(leftVals):    
                    fineright()
                    forward(2)
                elif within15(rightVals):
                    fineleft()
                    forward(2)
                else:
                    forward(1)

            elif REACT:

                actionPlan = detectObstacles((leftVals, rightVals, frontVals))

                if actionPlan == "180":
                    turn180()
                elif actionPlan == "R90":
                    turn90()
                elif actionPlan == "L90":
                    turn90("counterclockwise")
                else:
                    forward(1)


        elif test:
            viewSensors(SENSORS)
            
        # sleep to enforce loop rate
        r.sleep()
        

        
    
        

