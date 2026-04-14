from modular.URDF_writer import *

is_floating_base = True

with suppress_stdout():

    # create UrdfWriter object and joint map to store homing values
    urdf_writer = UrdfWriter(speedup=True, floating_base=is_floating_base)
    homing_joint_map = {}

    # add mobile base
    urdf_writer.add_module('concert/mobile_platform_concert.json', module_name='mobile_base')

    if is_floating_base:
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

    # big yaw (a.k.a. ralla)
    data = urdf_writer.add_module('experimental/module_joint_yaw_XL_concert.json', offsets={'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    homing_joint_map[str(data['name'])] = 0.0

    # prismatic joint
    data = urdf_writer.add_module('experimental/module_joint_prismatic_concert.json', offsets={'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    homing_joint_map[str(data['name'])] = 0.0

    data = urdf_writer.add_module('experimental/module_hub_prismatic_cart_concert.json', module_name='hub_prismatic_cart', offsets={'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    homing_joint_map[str(data['name'])] = 0.0

    # # Left mounted interface
    data = urdf_writer.select_module_from_name('hub_prismatic_cart_con4')

    data = urdf_writer.add_module('concert/module_joint_elbow_B_concert.json')
    homing_joint_map[str(data['name'])] = 0.0

    #add a 30cm passive link
    data = urdf_writer.add_module('concert/module_link_straight_300_concert.json')

    data = urdf_writer.add_module('concert/module_joint_elbow_B_concert.json')
    homing_joint_map[str(data['name'])] = 0.0
    
    #add a 30cm passive link
    data = urdf_writer.add_module('concert/module_link_straight_300_concert.json')

    data = urdf_writer.add_module('concert/module_joint_elbow_B_concert.json')
    homing_joint_map[str(data['name'])] = 0.0

    data = urdf_writer.add_module('concert/passive_end_effector_panel.json')

    # # Right mounted interface
    # data = urdf_writer.select_module_from_name('hub_prismatic_cart_con2')

    # Top mounted interface
    # data = urdf_writer.select_module_from_name('hub_prismatic_cart_con3')


write_file_to_stdout(urdf_writer, homing_joint_map)