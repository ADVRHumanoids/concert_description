from modular.URDF_writer import *

# If launched with --quiet flag, suppress stdout (e.g. for use in ROS2 launch files)
cli_args = UrdfWriter.parse_generator_cli_args()
quiet_mode = cli_args.quiet

# create UrdfWriter object and joint map to store homing values
urdf_writer = UrdfWriter(speedup=True, floating_base=True, quiet=quiet_mode)
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


urdf_writer.write_file_to_stdout(homing_joint_map)