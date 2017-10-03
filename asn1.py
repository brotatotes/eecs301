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

        go_to_default()

        raw_input("enter loop? ")

    else: # testing area
        test = True

        raw_input("loop? ")
    
    # Sensor setup
    IR1 = 1
    IR2 = 2
    DMS = 3

    # IR 1 is port 1
    # IR 2 is port 2

    # control loop running at X Hz
    r = rospy.Rate(1) # 1000hz
    # go_to_default()
    
    while not rospy.is_shutdown():
        
        # loop for each mode
        if asn0:
            irs[:len(irs)-1] = irs[1:]
            irs[-1] = getSensorValue(IR1)
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

        elif test:
            ir1 = getSensorValue(IR1)
            ir2 = getSensorValue(IR2)
            dms = getSensorValue(DMS)
            # print "IR 1: " + str(ir1)
            # print "IR 2: " + str(ir2)
            # print "Difference: " + str(abs(ir1 - ir2))
            # print 
            print "DMS: " + str(getSensorValue(DMS))
            
        # sleep to enforce loop rate
        r.sleep()
        

        
    
        

