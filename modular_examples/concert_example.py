from modular.URDF_writer import *

with suppress_stdout():

    # create UrdfWriter object and joint map to store homing values
    urdf_writer = UrdfWriter(speedup=True, floating_base=True)
    homing_joint_map = {}

    # add mobile base
    urdf_writer.add_mobile_platform()

    # leg + wheel 1
    data = urdf_writer.select_module_from_name('mobile_base_con1')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert_ORANGE_B.yaml', 
                                        steering_filename='concert/module_steering_concert_ORANGE_B.yaml', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = {'angle': 1.57}
    homing_joint_map[str(wheel_data['lastModule_name'])] = {'angle': 0.0}

    # leg + wheel 2
    data = urdf_writer.select_module_from_name('mobile_base_con2')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert_ORANGE_B.yaml', 
                                        steering_filename='concert/module_steering_concert_ORANGE_B.yaml', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = {'angle': -1.57}
    homing_joint_map[str(wheel_data['lastModule_name'])] = {'angle': 0.0}

    # leg + wheel 3
    data = urdf_writer.select_module_from_name('mobile_base_con3')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert_ORANGE_B.yaml', 
                                        steering_filename='concert/module_steering_concert_ORANGE_B.yaml', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = {'angle': -1.57}
    homing_joint_map[str(wheel_data['lastModule_name'])] = {'angle': 0.0}

    # leg + wheel 4
    data = urdf_writer.select_module_from_name('mobile_base_con4')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert_ORANGE_B.yaml', 
                                        steering_filename='concert/module_steering_concert_ORANGE_B.yaml', 
                                        angle_offset=0.0)
    homing_joint_map[str(steering_data['lastModule_name'])] = {'angle': 1.57}
    homing_joint_map[str(wheel_data['lastModule_name'])] = {'angle': 0.0}

    # manipulator
    data = urdf_writer.select_module_from_name('mobile_base_con5')

    # J1
    data = urdf_writer.add_module('concert/module_joint_concert_elbow_ORANGE_B.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.5}

    # J2
    data = urdf_writer.add_module('concert/module_joint_concert_elbow_ORANGE_B.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.5}

    # J3
    data = urdf_writer.add_module('concert/module_joint_concert_yaw_ORANGE_B.yaml', 0, False)
    homing_joint_map[data['lastModule_name']] = {'angle': 0.0}

    # J4
    data = urdf_writer.add_module('concert/module_joint_concert_elbow_ORANGE_B.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.5}

    # J5
    data = urdf_writer.add_module('concert/module_joint_concert_elbow_ORANGE_B.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.5}

    # gripper
    urdf_writer.add_simple_ee(0.0, 0.0, 0.2, 0.0)


write_file_to_stdout(urdf_writer, homing_joint_map)