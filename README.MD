documentation can be found at https://www.dji.com/sg/robomaster-s1/programming-guide
training package: https://drive.google.com/drive/folders/1GdPvwqAJcHHJWAeqxQEZMMZcUPQQJIz8?usp=sharing

this is coded in robomaster app, not python sdk, so a normal python file will kill itself when you try running it

<h3>quick documentation that might be needed during comps</h3> 
- chassis_ctrl.rotate_with_degree(rm_define.clockwise/rm_define.anticlockwise, 0-360)
- chassis_ctrl.move_with_distance(angle[-180/180], distance(m))
- chassis_ctrl.move_with_speed(0.5, 0, 0) #speed can also be tuned. its x axis forward (m/s), y axis right (m/s), speed roatation(ignore unless swerving). this is under block 15 of chassis_ctrl in the programming guide link above. here cuz took me forever to find
- gimbal_ctrl.recenter() (cos move_with_speed relies on absolute position, not relative)
- move arm to absolute position: robotic_arm_ctrl.moveto(x_coordinate, y_coordinate) (ts is in mm)
- move arm to relative position: robotic_arm_ctrl.move(x_coordinate, y_coordinate)
- distance from wall = ir_distance_sensor_ctrl.get_distance_info(1) (for some reason this bs is in cm while literally everything else is in m)
- led_ctrl.set_bottom_led(rm_define.armor_bottom_all, R, G, B, rm_define.effect_always_on)

<h3> stupid reminders </h3>
- rotate_with_degree and move_with_distance is in meters. based on current robot heading
- move_with_speed is X FORWARD and Y RIGHT. ts based on gimbal so remember to reset
- move_with_speed, x and y are m/s but speed is degree/s
- arm units are in mm
- ir units are in cm
