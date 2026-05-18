'''
print ir values for calibration
'''

gripper_ctrl.open()
ir_distance_sensor_ctrl.enable_measure(1)

while True:
    print(ir_distance_sensor_ctrl.get_distance_info(1))
    time.sleep(0.1)
