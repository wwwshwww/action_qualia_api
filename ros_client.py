import roslibpy as rlp
import numpy as np
import math
import quaternion # numpy-quaternion

import matplotlib.pyplot as plt

class RosClient():
    def __init__(self, master_name :str, port :int):
        self.master_name = master_name
        self.port = port
        self.client = rlp.Ros(self.master_name, port=self.port) #rlp.Ros('10.244.1.117', port=9090)\
        self.client.on_ready(lambda: print('Is ROS connected: ', self.client.is_connected))
        self.client.run()

        self.services = {} # {service_name: rlp.Service}
        # {service_name: {'service': rlp.Service, 'args': {args_key: []}}} <- botsu for now

    def register_servise(self, service_name: str, service_type: str):
        self.services[service_name] = rlp.Service(self.client, service_name, service_type)

    def call_service(self, service_name: str, args=None):
        srv = self.services[service_name]
        req = rlp.ServiceRequest(args)
        return srv.call(req)

class MobileSystem_2D():
    def __init__(self, ros_client: RosClient):
        self.ros = ros_client
        self.ros.register_servise('/dynamic_map', 'nav_msgs/GetMap')

        self.map_listener = rlp.Topic(self.ros.client, '/map', 'nav_msgs/OccupancyGrid')
        self.map_listener.subscribe(self._update_map)
        
        self.position = np.zeros(2) # [x,y] only 2d map
        self.orientation = np.quaternion(0,0,0,1) # (x,y,z,w) of quaternion
        self.odom_listener = rlp.Topic(self.ros.client, '/odom', 'nav_msgs/Odometry')
        self.odom_listener.subscribe(self._update_odometry)

    # def get_map(self) -> (dict, dict, np.array):
    #     res = self.ros.call_service('/dynamic_map')
    #     info = res['map']['info']
    #     header = res['map']['header']
    #     width = info['width']
    #     height = info['height']
    #     data = np.array(res['map']['data']).reshape([width,height])
    #     return info, header, data

    # need to make to be changed design pattern 'update_**'
    def _update_map(self, message):
        self.map_header = message['header']
        self.map_info = message['info']
        self.map_padsize_x = (self.map_info['width']-1)//2
        self.map_padsize_y = (self.map_info['height']-1)//2
        self.map = np.array(message['data']).reshape([self.map_info['height'],self.map_info['width']])

    def _update_odometry(self, message):
        pos = message['pose']['pose']['position']
        ori = message['pose']['pose']['orientation']
        self.position[0], self.position[1] = (pos['x'], pos['y'])
        self.orientation = np.quaternion(ori['x'], ori['y'], ori['z'], ori['w'])

    # perhaps nether need to create class of pose calculating
    @staticmethod
    def get_base_pose_from_relative(from_position, from_orientation: np.quaternion, to_position, to_orientation: np.quaternion) -> tuple:
        from_pos_q = np.quaternion(0,from_position[0],from_position[1],from_position[2])
        ## rotate angle (alpha,beta,gamma):(atan(z/y),atan(x/z),atan(x/y))
        to_angle = (math.atan2(to_position[2], to_position[1]), math.atan2(to_position[0], to_position[2]), math.atan2(to_position[0],to_position[1]))
        to_angle_q = quaternion.from_euler_angles(to_angle)
        to_pos_q = np.quaternion(0, to_position[0], to_position[1], to_position[2])
        # from_vec_dq = [1, from_pos_q]
        to_dq = [to_angle_q, 0.5*to_pos_q*to_angle_q]
        goal_pos_dq = [1, to_dq[0]*from_pos_q*to_dq[0].conj()+to_pos_q]
        goal_pos_orient_dq = [1, from_orientation*goal_pos_dq[1]*from_orientation.conj()]
        
        position = np.array([goal_pos_orient_dq[1].x, goal_pos_orient_dq[1].y, goal_pos_orient_dq[1].z])
        orientation = to_orientation*from_orientation
        ## returnable (np.array, np.quaternion)
        return position, orientation

    def get_base_pose_from_body(self, position=(0,0), orientation=(0,0,0,1)):
        return self.get_base_pose_from_relative(self.position, self.orientation, position, orientation)

    # map img's (i,j) to base map's (x,y)
    def get_coordinates_from_map(self, ij: tuple) -> tuple:
        return ij[1] - self.map_padsize_x, ij[0] - self.map_padsize_y

    # base map's (x,y) to base map's (i,j)
    def get_index_from_coordinates(self, xy: tuple) -> tuple:
        return xy[1] + self.map_padsize_y, xy[0] - self.map_padsize_x

    # take as relative coordinates
    def go_to(self, x, y):
        a = 0
        
    def send_to_goal(self, position: tuple, orientation: tuple):
        a=0

def main():
    rc = RosClient('10.244.1.117', 9090) # 
    rc.register_servise('/dynamic_map', 'nav_msgs/GetMap')
    print(rc.call_service('/dynamic_map'))

    rc.client.terminate()

if __name__ == '__main__':
    main()