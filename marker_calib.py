'''
steps
1. tune arm init position: find ideal coordinates where marker can be seen from far
2. tune stop distance
3. test angle correction
'''

speed_pid = PIDCtrl()
angular_pid = PIDCtrl()

picked_up = False
endRun = False

def initialise():
    speed_pid.set_ctrl_params(5, 0, 0)
    angular_pid.set_ctrl_params(150, 0, 0)

    chassis_ctrl.set_rotate_speed(100)

    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(3)
    
    gimbal_ctrl.recenter()

    ir_distance_sensor_ctrl.enable_measure(1)

    if robotic_arm_ctrl.get_position()[1] < 0: # to prevent arm from jamming if never reset properly
        robotic_arm_ctrl.move(0, 0-robotic_arm_ctrl.get_position()[1], wait_for_complete=True)
    robotic_arm_ctrl.recenter(wait_for_complete=True)
    gripper_ctrl.open()
    robotic_arm_ctrl.move(0, 25, wait_for_complete=True)
    robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True)
    # optimal for pickup is 185, -65
    # last actually tested 200, -10???
    # last ran: 200, -70


def set_led(R, G, B):
    led_ctrl.set_bottom_led(rm_define.armor_bottom_front, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_left, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_right, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_back, R, G, B, rm_define.effect_always_on)

def track_marker():
    """
    Adjust robot's position until in front of marker and return marker id.
    Might want to consider having different thresholds for different ids.
    """
    global distance_from_marker
    marker_id = 0
    threshold = 30  # Distance threshold (to be tuned)

    marker_info = vision_ctrl.get_marker_detection_info()
    print("Marker found" if len(marker_info) > 2 else "No marker")
    if len(marker_info) > 2:
        marker_id = marker_info[1]
        print(marker_id)

        distance_from_marker = ir_distance_sensor_ctrl.get_distance_info(1)
        speed_pid.set_error(distance_from_marker)
        x_offset = marker_info[2] - 0.5
        angular_pid.set_error(x_offset)

        while distance_from_marker > threshold:
            chassis_ctrl.move_with_speed(0.5, 0, angular_pid.get_output())
            time.sleep(0.005)

            marker_info = vision_ctrl.get_marker_detection_info()
            distance_from_marker = ir_distance_sensor_ctrl.get_distance_info(1)
            speed_pid.set_error(distance_from_marker)
            if len(marker_info) > 2:
                # have to frikin check another time in case image is cut of and it throws some out of index error
                x_offset = marker_info[2] - 0.5
                angular_pid.set_error(x_offset)

            print("Distance", distance_from_marker)

        print("Final distance from marker", distance_from_marker)
        chassis_ctrl.stop()
        print("Reached marker")
        return marker_info[1]
    
    else:
        # Move forward until marker is detected
        chassis_ctrl.move_with_speed(0.4)
        return 0

def moveForwardUntilWall(threshold):
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    
    while distance > threshold:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(0.5, 0, 0)
        print("Distance: ", distance)
        time.sleep(0.05)
        
    chassis_ctrl.stop()

def get_nearest_marker():
    marker = vision_ctrl.get_marker_detection_info()
    print(marker)
    if len(marker) <= 1:
        return None

    marker_count = marker[0]
    largest_width = 0
    nearest_marker = []

    for i in range(0, marker_count):
        base = 1 + i * 5

        marker_id = marker[base]
        x = marker[base + 1]
        y = marker[base + 2]
        w = marker[base + 3]
        h = marker[base + 4]

        print("Marker:", marker_id)
        print("Width:", w)

        if w > largest_width:
            largest_width = w
            nearest_marker = [marker_id, x, y, w, h]

    return nearest_marker

def main():
    initialise()
    time.sleep(1)
    marker_id = track_marker()
    while marker_id == 0:
        marker_id = track_marker()
    print("Marker id:", marker_id)

main()
