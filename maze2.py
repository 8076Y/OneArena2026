# 8076 yay! OneArena 2026 Maze 2 
import time


# global thresholds to tune
wallDistThreshold = 25
deadEndThreshold = 22

# pid for line tracking
speed_pid = PIDCtrl()
angular_pid = PIDCtrl()

# global flags
endRun = False
"""
Example Usage:

(Inside function)
    global endRun
    endRun = True
    print('End of run')

(Inside main)
    while not endRun:
"""

def initialise():
    # custom pid for line tracking
    speed_pid.set_ctrl_params(100, 0, 0)
    angular_pid.set_ctrl_params(450, 0, 0)

    # default rotation speed
    chassis_ctrl.set_rotate_speed(150)

    # track the blue guide line
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)

    # reset gimbal
    gimbal_ctrl.recenter()

    # enable IR distance sensor
    ir_distance_sensor_ctrl.enable_measure(1)

    # reset arm and gripper to initial positions (arm down, gripper open)
    if robotic_arm_ctrl.get_position()[1] < 0: # to prevent arm from jamming if never reset properly
        robotic_arm_ctrl.move(0, 0-robotic_arm_ctrl.get_position()[1], wait_for_complete=True)
    robotic_arm_ctrl.recenter(wait_for_complete=True)
    gripper_ctrl.open()
    robotic_arm_ctrl.move(0, 25, wait_for_complete=True)
    robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True)

def set_led(R, G, B):
    led_ctrl.set_bottom_led(rm_define.armor_bottom_front, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_left, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_right, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_back, R, G, B, rm_define.effect_always_on)


# ------------------------ movement helper functions ------------------------

def moveForwardUntilWall(threshold):
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    
    while distance > threshold:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(1, 0, 0)
        print("Distance: ", distance)
        time.sleep(0.05)
        
    chassis_ctrl.stop()

def escapeDeadEnd():
    chassis_ctrl.stop()
    set_led(255, 0, 0)
    chassis_ctrl.move_with_speed(-0.3, 0, 0) # back off the wall for clearance
    time.sleep(0.4)
    chassis_ctrl.stop()
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 180)
    set_led(0, 255, 0)

def searchForLine():
    chassis_ctrl.stop()
    set_led(255, 100, 0) # orange 
    chassis_ctrl.set_rotate_speed(40) # slow so we dont overshoot too far
    for _ in range(0, 24): # 360/15
        chassis_ctrl.rotate_with_degree(rm_define.clockwise, 15)
        if vision_ctrl.get_line_detection_info()[1] == 1:
            chassis_ctrl.set_rotate_speed(150) # restore default
            set_led(0, 255, 0)
            return True
    chassis_ctrl.set_rotate_speed(150)
    set_led(0, 0, 0)
    return False



# --------------------------- LINE TRACKING ---------------------------

def pleasewin(exit_dist=wallDistThreshold, lost_timeout=5):
    set_led(0, 255, 0)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    media_ctrl.exposure_value_update(rm_define.exposure_value_small) # makes frame darker so blue line pops 

    base_speed = 0.5
    settle_thresh = 0.06 # when goes back below this = bend finished
    turn_thresh = 0.18 # above this = bending
    in_turn = False
    last_good_steer = 0
    last_seen = time.time()

    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    while True:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        lineInfo = vision_ctrl.get_line_detection_info()
        if lineInfo[1] == 1: # single clean line
            last_seen = time.time()
            x_offset = lineInfo[14] - 0.5 # 4th point, lookahead
            angular_pid.set_error(x_offset)
            steer = angular_pid.get_output()
            last_good_steer = steer
            chassis_ctrl.move_with_speed(base_speed, 0, steer)

            if abs(x_offset) > turn_thresh:
                in_turn = True
            elif in_turn and abs(x_offset) < settle_thresh: # cleared the vertex
                in_turn = False
        else:
            if distance < deadEndThreshold:
                escapeDeadEnd();
                last_seen = time.time()
            
            else:
                if time.time() - last_seen > lost_timeout:
                    # lost line for too long
                    if not searchForLine():
                        break # just reset
                    last_seen = time.time()


            # dashed line or lost: keep curving to go over gap
            chassis_ctrl.move_with_speed(base_speed * 0.6, 0, last_good_steer * 0.6)
        time.sleep(0.05)
    chassis_ctrl.stop()
    vision_ctrl.disable_detection(rm_define.vision_detection_line)


# ---------------------------- MAIN ----------------------------

def main():
    initialise()
    pleasewin()

main()