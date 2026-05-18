''' movement functions '''
'''
    holonomic drive (move robot in a certain angle for a certain distance, while facing forward): chassis_ctrl.move_with_distance(angle, distance)
    move arm to absolute position: robotic_arm_ctrl.moveto(x_coordinate, y_coordinate)
    move arm to relative position: robotic_arm_ctrl.move(x_coordinate, y_coordinate)
    rotate on the spot: chassis_ctrl.rotate_with_degree(direction, angle)
    control all speeds: chassis_ctrl.rotate_with_degree(fb_speed, lr_speed, rotational_speed)
'''



picked_up = False
pickup_params = []

# Vision Markers
# CHANGE HERE
luggage1 = 14
luggage2 = 15
luggage3 = 16
luggage4 = 17
dropoff = 13
left = 12
right = 11
intersection1 = 8
intersection2 = 47

VM_List_picked_up = [luggage1, luggage2, luggage3, luggage4, left, right, intersection1, intersection2, 13] #the list of valid vision markers
VM_List_no_picked_up = [luggage1, luggage2, luggage3, luggage4, left, right, intersection1, intersection2]

curr_luggage = 0
num_luggage1 = 0
num_luggage2 = 0

distance = 1900 #set to a random large number
def start():
    while (not gripper_ctrl.is_open()):
        gripper_ctrl.open()
    robotic_arm_ctrl.moveto(150, 40, wait_for_complete=True)
    gripper_ctrl.update_power_level(2) # max gripping power
    chassis_ctrl.set_rotate_speed(60)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
    while (not gripper_ctrl.is_open()):
        gripper_ctrl.open()
    global picked_up
    global pickup_params
    global distance
    see_VM()

def drop():
    global picked_up
    global pickup_params
    global distance
    print("DROPPING")
    dist = 0.0
    chassis_ctrl.stop()
    print("FUCK")
    if picked_up == True:
        print("FUKC")
        # move forward into luggage deposit area
        #chassis_ctrl.move_with_distance(180, dist)
        # lower claw
        robotic_arm_ctrl.move(70, -100, wait_for_complete=True)
        while (not gripper_ctrl.is_open()):
            gripper_ctrl.open()
       
         # open claw
      
        print("Dropped")
        picked_up = False
        # reset claw position
        robotic_arm_ctrl.recenter()
        # move backwards to prevent hitting luggage upon reversing
        chassis_ctrl.move_with_distance(180, 0.1)
        # reverse robot
        chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 180)
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
    else:
        print("Nothing to drop")
    picked_up = False

def see_and_pickup(vm):
    global distance
    global picked_up
    marker_List = []
    # PID object for centering robot to vision marker
    pid_Centralise_Marker = PIDCtrl()
    # lower claw to default position
    # TO BE CALIBRATED
    chassis_ctrl.stop()
    if picked_up == False:
        robotic_arm_ctrl.moveto(200, -150, wait_for_complete=True)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(3)
    ir_distance_sensor_ctrl.enable_measure(1)
    pid_Centralise_Marker.set_ctrl_params(80, 1, 60)
    chassis_ctrl.set_trans_speed(1.2)
    state = True
    error = 100
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    while state:
        marker_List = vision_ctrl.get_marker_detection_info()
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        print(marker_List, distance, error)
        if (len(marker_List)>2):

            x = marker_List[marker_List.index(vm) +1]
            error = x-0.5
            #value to be calibrated
            pid_Centralise_Marker.set_error(error)
            pid_output = pid_Centralise_Marker.get_output()
            
            if distance > 12:
                x_speed = 0.1
            else:
                x_speed = 0
            if abs(error)>0.03:
                chassis_ctrl.move_with_speed(0, 0, pid_output)
            else:
                chassis_ctrl.move_with_speed(x_speed, 0, 0)
                print("Moving with speed", x_speed)
                print("Distance is ", distance)
        else:
            chassis_ctrl.move_with_speed(0.4, 0, 0)
            print("Moving forward")
    

        if (distance >0 and distance <12 and abs(error)<0.3):
            print("Moving to pick up, distance is", distance)
            chassis_ctrl.move_with_distance(0, 0.1)
            print("Moved forward by 0.10")
            chassis_ctrl.stop()
            print("Ready to Pick Up")
            state = False

    #reset distance
    distance = 100
    # fully open claw just in case, not really necessary
    while (not gripper_ctrl.is_open()):
        gripper_ctrl.open()
    # raise claw by 8cm to grab top of luggage
    # TO BE CALIBRATED
    robotic_arm_ctrl.move(0, 75, wait_for_complete=True)
    global pickup_params
    pickup_params = robotic_arm_ctrl.get_position()
    while (not gripper_ctrl.is_closed()):
        gripper_ctrl.close()


    # raise claw by another 5cm after closing
    # TO BE CALIBRATED
    robotic_arm_ctrl.move(-100, 50, wait_for_complete=True)
    picked_up = True
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    # reverse robot
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 180)


def see_VM():
    global distance
    global curr_luggage
    marker_List = []
    pid_Centralise_Marker = PIDCtrl()
    robotic_arm_ctrl.recenter()
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(3)
    media_ctrl.exposure_value_update(rm_define.exposure_value_large)
    ir_distance_sensor_ctrl.enable_measure(1)
    pid_Centralise_Marker.set_ctrl_params(70, 1, 80)
    state = True
    direction = ''
    to_drop = False
    global picked_up_misaligned
    global picked_up
    while state:
        # turn at closer dist when luggage is picked up since ir sensor is further forward
        # MAY NEED TO BE CALIBRATED
        
        VM_List = VM_List_picked_up
        
        if (picked_up == True):
            # TURN EARLIER IF HOLDING LUGGAGE SINCE THEY GAVE LEEWAY
            min_dist = 25
        else:
            min_dist = 17
        
            
        marker_List = vision_ctrl.get_marker_detection_info()
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        print(marker_List, distance)
        lost_vm = True
        if (len(marker_List)>2 and marker_List[1] in VM_List):
            print(marker_List, distance)
            #first marker in list
            x = marker_List[2]
            if ((luggage1 in marker_List or luggage2 in marker_List or luggage3 in marker_List or luggage4 in marker_List or 13 in marker_List)):
                if (picked_up == False):
                    chassis_ctrl.stop()
                
                    if luggage1 in marker_List:
                        curr_luggage = 4
                        see_and_pickup(luggage1)
                    elif luggage2 in marker_List:
                        curr_luggage = 5
                        see_and_pickup(luggage2)
                    elif luggage3 in marker_List:
                        curr_luggage = 6
                        see_and_pickup(luggage3)
                    else:
                        curr_luggage = 7
                        see_and_pickup(luggage4)
                else:
                    if (13 in marker_List):
                        to_drop = True
                        print("to drop")
                        
            elif (13 in marker_List and 8 not in marker_List):
                print("to drop")
                to_drop = True
                print("to drop")
        

            elif (right in marker_List):
                print('here')
                direction = 'right'
            elif (left in marker_List):
                direction = 'left'

            elif (47 in marker_List):
               
                # QUESTION MARK
                # FIRST DECISION
                if (curr_luggage == 6 or curr_luggage == 7):
                    direction = 'right'
                else: 
                    direction = 'left'
         
            elif (8 in marker_List):
               
                # HEART 
                # SECOND DECISION
                if (curr_luggage == 5 or curr_luggage == 7):
                    direction = 'right'
                else: 
                    direction = 'left'
           
            else:
                pass


            error = x-0.5 #value to be calibrated
            pid_Centralise_Marker.set_error(error)
            pid_output = pid_Centralise_Marker.get_output()
            distance = ir_distance_sensor_ctrl.get_distance_info(1)
            
            
            # TUNE THIS SPEED TMR
            if (distance > min_dist and picked_up == False):
                x_speed = 0.3
            elif (distance > min_dist and picked_up == True):
                x_speed = 0.3
            else:
                x_speed = 0
            #if correct vision marker seen, move towards the vision marker
            chassis_ctrl.move_with_speed(x_speed, 0, pid_output)
            print("Correcting direction")
            lost_vm = False
        
       

        if (to_drop == True and distance >0 and distance <30):
            print("Dropping")
            drop()
            lost_vm = False
            to_drop = False
        elif (direction == 'left' and distance >0 and distance <min_dist):
            chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
            print("Rotating left")
            print("Rotation complete")
            direction = ''
            lost_vm = False
            time.sleep(0.5)
        elif (direction == 'right' and distance >0 and distance <min_dist):
            print("Rotating right")
            chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
            print("Rotation complete")
            direction = ''
            lost_vm = False
            time.sleep(0.5)
        if lost_vm:
            print("LOST VM")
            chassis_ctrl.move_with_speed(0.4, 0, 0)
           
start()


'''
IMPORTANT SHIT TO TEST WITH ACTUAL ROBOT

POSITION OF RAISED AND LOWERED CLAW
'''