import roslibpy as rlp

class RosClient():
    def __init__(self, master_name :str, port :int):
        self.master_name = master_name
        self.port = port
        self.client = rlp.Ros(self.master_name, port=self.port) #rlp.Ros('10.244.1.117', port=9090)\
        self.client.on_ready(lambda: print('is ROS connected: ', self.client.is_connected))
        self.client.run()

        self.services = {} # {service_name: rlp.Service}
        #botsu# {service_name: {'service': rlp.Service, 'args': {args_key: []}}}

    def register_servise(self, service_name: str, service_type: str):
        self.services[service_name] = rlp.Service(self.client, service_name, service_type)

    def call_service(self, service_name: str, args: list=None):
        srv = self.services[service_name]
        req = rlp.ServiceRequest(args)
        return srv.call(req)

def main():
    # rc = RosClient('10.244.1.176', 9090)
    rc = RosClient('localhost', 9090)
    rc.register_servise('/dynamic_map', 'nav_msgs/GetMap')
    print(rc.call_service('/dynamic_map'))

if __name__ == '__main__':
    main()