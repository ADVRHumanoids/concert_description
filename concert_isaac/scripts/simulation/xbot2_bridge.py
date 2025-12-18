import os 
import socket
import yaml
import torch

from isaaclab.assets import Articulation
from isaaclab.sensors.imu import Imu

class IsaacXBot2Bridge:

    def __init__(self, robot: Articulation, imu_sensors: dict[str, Imu], urdf_str: str = ''):

        # Socket for communication
        server_socket_path = os.getenv("ISAAC_XBOT2_BRIDGE_SOCK_ADDR", "/tmp/.xbot2_isaac/xbot2_isaac_server.sock")
        if os.path.exists(server_socket_path):
            os.unlink(server_socket_path)
        
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(server_socket_path)
        self.sock.setblocking(False)
        os.chmod(server_socket_path, 0o777)

        print(f"Server socket created at {server_socket_path}")

        # Set to keep track of client sockets
        self.client_sockets = set()

        # Save scene objects
        self.robot = robot
        self.imu_sensors = imu_sensors
        self.urdf_str = urdf_str

    def send_to_clients(self, time: float):

        robot = self.robot
        imu_sensors = self.imu_sensors

        if len(self.client_sockets) == 0:
            return # No clients connected

        # Broadcast robot state to all clients
        state_msg = {'type': 'state'}
        state_msg['time'] = time
        state_msg['q'] = robot.data.joint_pos.cpu().numpy().flatten().tolist()
        state_msg['dq'] = robot.data.joint_vel.cpu().numpy().flatten().tolist()
        state_msg['tau'] = robot.data.applied_torque.cpu().numpy().flatten().tolist()
        state_msg['k'] = robot.data.joint_stiffness.cpu().numpy().flatten().tolist()
        state_msg['d'] = robot.data.joint_damping.cpu().numpy().flatten().tolist()
        state_msg['qref'] = robot.data.joint_pos_target.cpu().numpy().flatten().tolist()
        state_msg['vref'] = robot.data.joint_vel_target.cpu().numpy().flatten().tolist()
        state_msg['tauref'] = robot.data.joint_effort_target.cpu().numpy().flatten().tolist()
        
        state_msg['imu'] = dict()
        for imu_name, imu_sensor in imu_sensors.items():
            imu_data = imu_sensor.data
            state_msg['imu'][imu_name] = {
                'quat_w': imu_data.quat_w.cpu().numpy().flatten().tolist(),
                'lin_acc_b': imu_data.lin_acc_b.cpu().numpy().flatten().tolist(),
                'ang_vel_b': imu_data.ang_vel_b.cpu().numpy().flatten().tolist(),
            }

        # Serialize message
        state_msg = yaml.dump(state_msg, default_flow_style=False)

        # Send to all clients, keep track of disconnected clients
        sockets_to_remove = []
        for cli_addr in self.client_sockets:
            try:
                self.sock.sendto(state_msg.encode(), cli_addr)
            except ConnectionRefusedError as e:
                print(f"Client at {cli_addr} disconnected.")
                sockets_to_remove.append(cli_addr)
            except Exception as e:
                print(f"Error sending state to {cli_addr}: {e}")

        for s in sockets_to_remove:
            self.client_sockets.remove(s)
        
    def recv_from_clients(self):
        robot = self.robot
        imu_sensors = self.imu_sensors
        # handle client connections
        try:
            # recv, if not message available, will raise BlockingIOError
            data, cli_addr = self.sock.recvfrom(4096)

            # consume buffer 
            while True:
                try:
                    data, cli_addr = self.sock.recvfrom(4096)
                except BlockingIOError:
                    break

            # decode data
            data = data.decode('utf-8')
            data = yaml.safe_load(data)
            data_type = data['type']
            if data_type == 'discovery':
                response = {'type': 'discovery'}
                response['joint_names'] = robot.joint_names
                response['imu_sensors'] = list(imu_sensors.keys())
                response['urdf'] = self.urdf_str
                try:
                    self.sock.sendto(yaml.dump(response, default_flow_style=True).encode('utf-8'), cli_addr)
                except Exception as e:
                    print(f"Error sending discovery response to {cli_addr}: {e}")
                self.client_sockets.add(cli_addr)
                print(f"Client at {cli_addr} connected.")

            elif data_type == 'control':
                joint_pos_ref = torch.tensor(data['q'], device=robot.device).unsqueeze(0)
                joint_vel_ref = torch.tensor(data['dq'], device=robot.device).unsqueeze(0)
                robot.set_joint_position_target(joint_pos_ref)
                robot.set_joint_velocity_target(joint_vel_ref)

            else:
                print(f"Unknown data type received: {data_type}")

        except BlockingIOError:
            pass