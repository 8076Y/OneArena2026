picked_up = False
pickup_params = []

run_time = 0

distance = 9999

open_door = 11
arc_turn = 12
reverse = 13
pickup = 14
stop = 15
ramp = 8
qn_mark = 47

pickup_threshold = 50
sign_threshold = 30

speed_pid = PIDCtrl() #vary speed as we approach marker
angular_pid = PIDCtrl() #correct the robot's angle to face the marker

def initialise():
    speed_pid.set_ctrl_params(5, 0, 0)
    angular_pid.set_ctrl_params(150, 0, 0) 
    chassis_ctrl.set_rotate_speed(100)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(3)
    ir_distance_sensor_ctrl.enable_measure(1)

    robotic_arm_ctrl.recenter(wait_for_complete=True)
    gripper_ctrl.open()
    robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True) #default position is arm down, gripper open

def track_and_pickup():
    global distance
    global picked_up
    gripper_ctrl.open()
    #Inch forward until IR distance is minimum
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    print("Distance from cone: ", distance)
    while distance > 10:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(0.1, 0, 0)
        print(distance)
    chassis_ctrl.stop()
    print("picking up")
    while (not gripper_ctrl.is_closed()):
        gripper_ctrl.close()
    robotic_arm_ctrl.moveto(200, -20)
    picked_up = True
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 180)

def s_curve_fn():
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 135)
    chassis_ctrl.move_with_distance(0, 0.9)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 135)

def dropfn():
    global picked_up
    #stops any ongoing movement of the robot's chassis,
    #ensuring the robot is stationary before it attempts to drop an object
    chassis_ctrl.stop()
    if picked_up == True: #if the robot is loaded
        #chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 85) #robot turn 85 degree anticlockwise
        #chassis_ctrl.move_with_distance(0, 0.35) #0.35 in meter
        robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True)
        #open the gripper - drop the item
        while (not gripper_ctrl.is_open()):
            gripper_ctrl.open()
        print("Dropped")
        picked_up = False
        robotic_arm_ctrl.recenter() #move robot's arm back to default position
        chassis_ctrl.move_with_distance(180, 0)
        chassis_ctrl.rotate_with_degree(rm_define.clockwise, 180)
    else:
        print("Nothing to drop")
    run_time_drop = tools.run_time_of_program()
    print(run_time_drop)

def arc_turnfn():
    for i in range(220):
        chassis_ctrl.move_with_speed(0.3, 0.3, 70)
    chassis_ctrl.stop()
    time.sleep(1)
    chassis_ctrl.move_with_distance(-180, 0.5)

def open_doorsfn():
    chassis_ctrl.set_trans_speed(0.2)
    print("Start pushing")
    while True:
        chassis_ctrl.move(-90)
        vision_ctrl.cond_wait(rm_define.cond_recognized_marker_number_three)
        break

def reversefn():
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 180)

def climb_rampfn():
    pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
    while pitch > -5:
        pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
        chassis_ctrl.move_with_speed(2.0, 0, 0)
    while pitch < -20:
        pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
        chassis_ctrl.move_with_speed(0.1, 0, 0)

def endfn():
    chassis_ctrl.stop()
    led_ctrl.set_flash(rm_define.armor_all ,10)
    media_ctrl.play_sound(rm_define.media_sound_count_down)
    #Change the LED color to RED to indicate a STOP status
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
    #Calculate and print programme runtime
    whole_programme_running_time = tools.timer_current() #retrieve the current value of timer
    tools.timer_ctrl(rm_define.timer_stop) #stops the timer
    print (whole_programme_running_time)
    overall_score = 3600 - round(whole_programme_running_time) #calculate score by subtracting from 3600 seconds (1hr)
    print (overall_score)

def track_marker():
    global distance
    marker_id = 0
    threshold = 0
    marker_info = vision_ctrl.get_marker_detection_info()
    if len(marker_info) > 2:
        marker_id = marker_info[1]
        print (marker_id)
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        speed_pid.set_error(distance)
        x_offset = marker_info[2] - 0.5
        angular_pid.set_error(x_offset)
        if marker_id == pickup:
            threshold = pickup_threshold
        else:
            threshold = sign_threshold
        while distance > threshold:
            chassis_ctrl.move_with_speed(0.5, 0, angular_pid.get_output())
            time.sleep(0.005)
            marker_info = vision_ctrl.get_marker_detection_info()
            distance = ir_distance_sensor_ctrl.get_distance_info(1)
            speed_pid.set_error(distance)
            x_offset = marker_info[2] - 0.5
            angular_pid.set_error(x_offset)
            print("dist", distance)

        chassis_ctrl.stop()
        print("at marker")
        return marker_id

    else:
        chassis_ctrl.move_with_speed(0.4) #move forward till a marker is spotted
        return 0


def main():
    initialise()
    while True:
        target = track_marker()
        print("done")
        if target == pickup:
            track_and_pickup()
            print("pickup")
        elif target == qn_mark:
            dropfn()
            print("drop")
        elif target == open_door:
            open_doorfn()
            print("door")
        elif target == arc_turn:
            arc_turnfn()
            print("arc")
        elif target == reverse:
            reversefn()
            print("reverse")
        elif target == stop:
            stopfn()
            print("stop")
        elif target == ramp:
            climb_ramp()
            print("ramp")
        elif target == 0:
            print("continue")
        else:
            print('wahoo')
main()