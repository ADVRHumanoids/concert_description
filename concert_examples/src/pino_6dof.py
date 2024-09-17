from modular.URDF_writer import *

with suppress_stdout():

    # create UrdfWriter object and joint map to store homing values
    urdf_writer = UrdfWriter(speedup=True)
    homing_joint_map = {}

    # urdf_writer.add_table()
    # urdf_writer.add_socket(0.150, 0.225, 0.0, 0.0)
    urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

    # J1
    data = urdf_writer.add_module('module_joint_yaw_ORANGE.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.0}

    # J2
    data = urdf_writer.add_module('module_joint_double_elbow_ORANGE.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.5}

    # J3
    data = urdf_writer.add_module('module_joint_yaw_ORANGE.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.0}

    # J4
    data = urdf_writer.add_module('module_joint_double_elbow_ORANGE.yaml', 0, False)
    homing_joint_map[data['lastModule_name']] = {'angle': 1.0}

    # J5
    data = urdf_writer.add_module('module_joint_yaw_ORANGE.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 0.0}

    # J6
    data = urdf_writer.add_module('module_joint_double_elbow_ORANGE.yaml', 0, False)
    homing_joint_map[str(data['lastModule_name'])] = {'angle': 1.5}


    # gripper
    # data = urdf_writer.add_module('module_gripper.yaml', 0, False)
    urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)


write_file_to_stdout(urdf_writer, homing_joint_map)