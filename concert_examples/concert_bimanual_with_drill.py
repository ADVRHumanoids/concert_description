from modular.URDF_writer import *

with suppress_stdout():

    # create UrdfWriter object and joint map to store homing values
    urdf_writer = UrdfWriter(speedup=True, floating_base=True)
    homing_joint_map = {}

    # add mobile base
    urdf_writer.add_module('concert/mobile_platform_concert.json', module_name='mobile_base')

    # leg + wheel 1
    data = urdf_writer.select_module_from_name('mobile_base_con1')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fl_rr.json')
    homing_joint_map[str(steering_data['name'])] = 0.0
    homing_joint_map[str(wheel_data['name'])] = 0.0

    # leg + wheel 2
    data = urdf_writer.select_module_from_name('mobile_base_con2')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fr_rl.json')
    homing_joint_map[str(steering_data['name'])] = 0.0
    homing_joint_map[str(wheel_data['name'])] = 0.0

    # leg + wheel 3
    data = urdf_writer.select_module_from_name('mobile_base_con3')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fr_rl.json')
    homing_joint_map[str(steering_data['name'])] = 0.0
    homing_joint_map[str(wheel_data['name'])] = 0.0

    # leg + wheel 4
    data = urdf_writer.select_module_from_name('mobile_base_con4')
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename='concert/module_wheel_concert.json', 
                                        steering_filename='concert/module_steering_concert_fl_rr.json')
    homing_joint_map[str(steering_data['name'])] = 0.0
    homing_joint_map[str(wheel_data['name'])] = 0.0

    # manipulator
    data = urdf_writer.select_module_from_name('mobile_base_con5')

    # Torso
    data = urdf_writer.add_module('concert/module_Y_torso_concert.json', module_name='torso')

    ##################################################################################
    # arm 1
    data = urdf_writer.select_module_from_name('torso_con2')

    # J1 
    data = urdf_writer.add_module('concert/module_joint_yaw_A_concert.json')
    homing_joint_map[data['name']] = 0.0

    # J2 
    data = urdf_writer.add_module('concert/module_joint_elbow_A_concert.json')
    homing_joint_map[str(data['name'])] = 0.5

    # J3 
    data = urdf_writer.add_module('concert/module_joint_yaw_B_concert.json')
    homing_joint_map[data['name']] = 0.0

    # add a 40cm passive link
    data = urdf_writer.add_module('concert/module_link_straight_400_concert.json')

    # J4
    data = urdf_writer.add_module('concert/module_joint_elbow_A_concert.json')
    homing_joint_map[str(data['name'])] = 1.9

    # J5
    data = urdf_writer.add_module('concert/module_joint_yaw_B_concert.json')
    homing_joint_map[str(data['name'])] = 0.0

    # add a 40cm passive link
    data = urdf_writer.add_module('concert/module_link_straight_300_concert.json')

    # J6
    data = urdf_writer.add_module('concert/module_joint_elbow_B_concert.json')
    homing_joint_map[str(data['name'])] = 1.3

    # drill
    urdf_writer.add_module('concert/module_drill_concert.json')

    # drillbit
    # urdf_writer.add_simple_ee(z_offset=0.30, radius=0.01, name='drillbit')
    urdf_writer.add_drillbit(length=0.27, radius=0.012, mass=0.1)

    ########################################################################
    # arm 2
    data = urdf_writer.select_module_from_name('torso_con3')

    # J1 
    data = urdf_writer.add_module('concert/module_joint_yaw_A_concert.json')
    homing_joint_map[data['name']] = 0.0

    # J2 
    data = urdf_writer.add_module('concert/module_joint_elbow_A_concert.json')
    homing_joint_map[str(data['name'])] = 0.5

    # J3 
    data = urdf_writer.add_module('concert/module_joint_yaw_B_concert.json')
    homing_joint_map[data['name']] = 0.0

    # add a 40cm passive link
    data = urdf_writer.add_module('concert/module_link_straight_400_concert.json')

    # J4
    data = urdf_writer.add_module('concert/module_joint_elbow_A_concert.json')
    homing_joint_map[str(data['name'])] = 1.9

    # J5
    data = urdf_writer.add_module('concert/module_joint_yaw_B_concert.json')
    homing_joint_map[str(data['name'])] = 0.0

    # add a 40cm passive link
    data = urdf_writer.add_module('concert/module_link_straight_300_concert.json')

    # J6
    data = urdf_writer.add_module('concert/module_joint_elbow_B_concert.json')
    homing_joint_map[str(data['name'])] = 1.3

    # drill
    urdf_writer.add_module('concert/module_drill_concert.json')

    # drillbit
    # urdf_writer.add_simple_ee(z_offset=0.30, radius=0.01, name='drillbit')
    urdf_writer.add_drillbit(length=0.27, radius=0.012, mass=0.1)

write_file_to_stdout(urdf_writer, homing_joint_map)
