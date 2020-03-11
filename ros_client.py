import roslibpy as rlp
import numpy as np

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

class MobileSystem():
    def __init__(self, ros_client: RosClient, map_getter='/dynamic_map'):
        self.ros = ros_client
        self.map_getter = map_getter
        self.ros.register_servise(self.map_getter, '/nav_msgs/GetMap')
        self.map_info, self.map_header, self.map = self.get_map()
        self.position = (0,0) # only 2d map
        self.orientation = (0,0,0,1) # (x,y,z,w) of quaternion

    def get_map(self) -> (dict, dict, np.array):
        res = self.ros.call_service(self.map_getter)
        info = res['map']['info']
        header = res['map']['header']
        width = info['width']
        height = info['height']
        data = np.array(res['map']['data']).reshape([width,height])
        return info, header, data

    def 
        

def main():
    rc = RosClient('10.244.1.117', 9090) # 
    rc.register_servise('/dynamic_map', 'nav_msgs/GetMap')
    print(rc.call_service('/dynamic_map'))

    rc.client.terminate()

if __name__ == '__main__':
    main()