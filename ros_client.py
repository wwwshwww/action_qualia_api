import roslibpy as rlp
import roslibpy.actionlib
import numpy as np
import math
import quaternion # numpy-quaternion
from collections import deque
import time
from threading import Timer

import matplotlib.pyplot as plt

class RosClient():
    def __init__(self, master_name :str, port :int):
        self.master_name = master_name
        self.port = port
        self.client = rlp.Ros(self.master_name, port=self.port) #rlp.Ros('10.244.1.117', port=9090)\
        self.client.on_ready(lambda: print('Is ROS connected: ', self.client.is_connected))
        self.client.run()

        self.services = {} # {service_name: rlp.Service}
        #botsu# {service_name: {'service': rlp.Service, 'args': {args_key: []}}}

    def register_servise(self, service_name: str, service_type: str):
        self.services[service_name] = rlp.Service(self.client, service_name, service_type)

    def call_service(self, service_name: str, args: list=None):
        srv = self.services[service_name]
        req = rlp.ServiceRequest(args)
        return srv.call(req)

## FCFS (First Come First Served）
class ActionScheduler(Timer):
    STATUS_READY_TO_START = 0b01
    def __init__(self, ros_client: rlp.Ros, server_name: str, action_name: str, callback: callable, queue_size: int=100, rate: float=0.05, args: list=None, kwargs: dict=None): 
        Timer.__init__(self, rate, self.run, args, kwargs)
        self.ros_client = ros_client
        self.action_client = rlp.actionlib.ActionClient(ros_client, server_name, action_name)
        self.current_goal: rlp.actionlib.Goal = None
        self.callback = callback
        self.thread = None
        self.goal_queue = deque(maxlen=queue_size)
        self.rate = rate
        self.state = ActionScheduler.STATUS_READY_TO_START ## 01: queue is empty, 10: task is actived
        
    def _update_state(self):
        is_emp = 1 if len(self.goal_queue) == 0 else 0
        is_fin = 0
        if self.current_goal is not None:
            is_fin = 0 if self.current_goal.is_finished else 1
        self.state = is_emp|(is_fin<<1)

    def _check_task(self):
        self._update_state()
        self.thread = Timer(self.rate, self._check_task)
        self.thread.start()

        if not self.state&1 and not self.state>>1&1:
            self.current_goal = self.goal_queue.popleft()
            self._update_state()
            self.current_goal.send(self._finish)

    def _finish(self, result):
        self._update_state()
        self.callback(result)

    def append_goal(self, goal_message):
        goal = rlp.actionlib.Goal(self.action_client, goal_message)
        self.goal_queue.append(goal)
        self._update_state()

    def run(self):
        self._check_task()

    def cancel(self):
        if self.thread is not None:
            self.thread.cancel()
            self.thread.join()
            del self.thread

## make message that has subscribed synchronized in time approximate, before call a callback function
class TimeSynchronizer():
    class Subscriber():
        def __init(self, sync: TimeSynchronizer ,listener: rlp.Topic, queue_size: int):
            self.sync = sync
            self.listener = listener
            self.queue = deque(maxlen=queue_size)
            self.listener.subscribe(self.cb)
        
        def cb(self, message):
            self.queue.append(message)
            self.sync.synchronize()

    def __init__(self, topics: list, callback: callable, queue_size: int, allow_headerless: float=None):
        self.listeners = [TimeSynchronizer.Subscriber(self, s, queue_size) for s in topics]
        self.callback = callback
        self.allow_headerless = allow_headerless
        self.queue = deque(maxlen=queue_size)

    def get_time(self, message):
        return message['header']['stamp']['secs']*(10**9)+message['header']['stamp']['nsecs']

    def synchronize(self):
        if not all([len(s.queue) for s in self.listeners]):
            return

        lis_len = len(self.listeners)
        di = {i:t for i,t in zip(range(lis_len), map(self.get_time, [l.queue[0] for l in self.listeners]))}
        cri_time_i = max(di, key=di.get)
        cri_time = di[cri_time_i]
        allow_time = cri_time + self.allow_headerless
        del di[cri_time_i]

        result = [None for x in range(lis_len)]
        result[cri_time_i] = self.listeners[cri_time_i].queue[0]

        for k in di.keys():
            # al = np.array([i for i in map(self.get_time, self.listeners[k].queue)])
            tq = deque()
            for i in range(len(self.listeners[k].queue)):
                nt = self.get_time(self.listeners[k].queue[i])
                if nt <= allow_time:
                    tq.append(nt)
                else:
                    break
            
            arr = np.abs(np.array(tq)-cri_time)
            min_i = np.argmin(arr)
            result[k] = self.listeners[k].queue[min_i]
            for i in range(len(tq)):
                self.listeners[k].queue.popleft()
        
        self.callback(*result)

class MobileSystem_2D():
    def __init__(self, ros_client: rlp.Ros):
        self.ros_client = ros_client
        self.mb_scheduler = ActionScheduler(self.ros_client, '/move_base', 'move_base_msgs/MoveBaseAction', self.ppp)
        # self.ros.register_servise('/dynamic_map', 'nav_msgs/GetMap')

        self.is_get_map = False
        self.map_listener = rlp.Topic(self.ros_client, '/map', 'nav_msgs/OccupancyGrid')
        self.map_listener.subscribe(self._update_map)
        
        self.is_get_odom = False
        self.odom_listener = rlp.Topic(self.ros_client, '/odom', 'nav_msgs/Odometry')
        self.odom_listener.subscribe(self._update_odometry)

    @property
    def is_reached(self):
        return not self.mb_scheduler.state>>1

    ## need to make to be changed design pattern 'update_**'
    def _update_map(self, message):
        self.map_header = message['header']
        self.map_info = message['info']
        self.map_padsize_x = (self.map_info['width']-1)//2
        self.map_padsize_y = (self.map_info['height']-1)//2
        self.map = np.array(message['data']).reshape([self.map_info['height'],self.map_info['width']])
        self.is_get_map = True

    def _update_odometry(self, message):
        print(message)
        pos = message['pose']['pose']['position']
        ori = message['pose']['pose']['orientation']
        self.position = np.array([pos['x'], pos['y']])
        self.orientation = np.quaternion(ori['w'], ori['x'], ori['y'], ori['z'])
        self.is_get_odom = True

    def wait_for_ready(self, timeout=10.0):
        sec = 0.001
        end_time = time.time() + timeout
        while True:
            if self.is_get_odom and self.is_get_map:
                break
            elif time.time()+sec >= end_time:
                self.ros_client.terminate()
                raise Exception('Timeout you can\'t get map or odometry.')
            time.sleep(sec)

    ## perhaps nether need to create class of pose calculating
    @staticmethod
    def get_base_pose_from_relative(from_position, from_orientation: np.quaternion, to_position, to_orientation: np.quaternion) -> (np.array, np.quaternion):
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

    def get_base_pose_from_body(self, position, orientation=np.quaternion(1,0,0,0)) -> (np.array, np.quaternion):
        return self.get_base_pose_from_relative(self.position, self.orientation, position, orientation)

    ## map img's (i,j) to base map's (x,y)
    def get_coordinates_from_map(self, ij: tuple) -> tuple:
        return ij[1] - self.map_padsize_x, ij[0] - self.map_padsize_y

    ## base map's (x,y) to base map's (i,j)
    def get_index_from_coordinates(self, xy: tuple) -> tuple:
        return xy[1] + self.map_padsize_y, xy[0] - self.map_padsize_x

    def create_message_move_base_goal(self, position: tuple, orientation: np.quaternion) -> dict:
        message = {
            'target_pose': {
                'header': self.map_header,
                'pose': {
                    'position': {
                        'x': position[0],
                        'y': position[1],
                        'z': position[2]
                    },
                    'orientation': {
                        'x': orientation.x,
                        'y': orientation.y,
                        'z': orientation.z,
                        'w': orientation.w
                    }
                }
            }
        }
        return rlp.Message(message)

    ## set goal message that simple ahead pose
    def set_goal(self, x, y, angle):
        rel_pos_2d = (x,y,0)
        rel_ori = quaternion.from_euler_angles(0, 0, math.atan2(x,y))
        pos, ori = self.get_base_pose_from_body(rel_pos_2d, rel_ori)
        mes = self.create_message_move_base_goal(pos, ori)
        self.mb_scheduler.append_goal(mes)
    
    def ppp(self, result):
        print(result)

    def start(self):
        self.mb_scheduler.start()

    def stop(self):
        self.mb_scheduler.cancel()

def main():
    rc = RosClient('10.244.1.117', 9090) # 
    # rc.register_servise('/dynamic_map', 'nav_msgs/GetMap')
    # print(rc.call_service('/dynamic_map'))
    ms = MobileSystem_2D(rc.client)
    ms.wait_for_ready()

    print(ms.map_header)

    rc.client.terminate()

if __name__ == '__main__':
    main()