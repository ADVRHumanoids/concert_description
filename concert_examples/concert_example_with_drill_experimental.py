from modular.URDF_writer import *

with suppress_stdout():

    # create UrdfWriter object and joint map to store homing values
    urdf_writer = UrdfWriter(speedup=True, floating_base=True)
    homing_joint_map = {}

    # add mobile base
    urdf_writer.add_mobile_platform()

    # leg + wheel 1
    data = urdf_writer.select_module_from_name('mobile_base_con1')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fl_rr.json', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = 0.0
    homing_joint_map[str(wheel_data['lastModule_name'])] = 0.0

    # leg + wheel 2
    data = urdf_writer.select_module_from_name('mobile_base_con2')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fr_rl.json', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = 0.0
    homing_joint_map[str(wheel_data['lastModule_name'])] = 0.0

    # leg + wheel 3
    data = urdf_writer.select_module_from_name('mobile_base_con3')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fr_rl.json', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = 0.0
    homing_joint_map[str(wheel_data['lastModule_name'])] = 0.0

    # leg + wheel 4
    data = urdf_writer.select_module_from_name('mobile_base_con4')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fl_rr.json', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = 0.0
    homing_joint_map[str(wheel_data['lastModule_name'])] = 0.0

    # manipulator
    data = urdf_writer.select_module_from_name('mobile_base_con5')

    # add a 60cm passive link
    data = urdf_writer.add_module('experimental/module_link_elbow_35_concert.json', 0, False)

    # J1
    data = urdf_writer.add_module('concert/module_joint_elbow_A_concert.json', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = -0.9

    # J2
    data = urdf_writer.add_module('concert/module_joint_yaw_A_concert.json', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = 0.0

    # add a 30cm passive link
    data = urdf_writer.add_module('experimental/module_link_straight_30_concert.json', 0, False)

    # J3
    data = urdf_writer.add_module('concert/module_joint_elbow_A_concert.json', 0, False)
    homing_joint_map[data['lastModule_name']] = -1.0

    # J4
    data = urdf_writer.add_module('concert/module_joint_yaw_B_concert.json', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = 0.0

    # add a 40cm passive link
    data = urdf_writer.add_module('experimental/module_link_straight_40_concert.json', 0, False)

    # J5
    data = urdf_writer.add_module('concert/module_joint_elbow_B_concert.json', 0, False)
    homing_joint_map[data['lastModule_name']] = 1.57

    # drill
    urdf_writer.add_module('concert/module_drill_concert.json', 0, False)

write_file_to_stdout(urdf_writer, homing_joint_map)