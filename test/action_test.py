import roslibpy
import roslibpy.actionlib
import time

def main():
    client = roslibpy.Ros('localhost', port=9090) #rlp.Ros('10.244.1.117', port=9090)\
    client.on_ready(lambda: print('Is ROS connected: ', client.is_connected))
    client.run()

    # client.get_action_servers(lambda s: print('servers:', s))

    # time.sleep(5)
    roslibpy.actionlib.DEFAULT_CONNECTION_TIMEOUT = 10
    action_client = roslibpy.actionlib.ActionClient(client,"/fibonacci","actionlib_tutorials/FibonacciAction")
    goal = roslibpy.actionlib.Goal(action_client,roslibpy.Message({'order': 8}))

    goal.on('feedback', lambda f: print(f['sequence']))
    goal.send()
    result = goal.wait()
    action_client.dispose()

    print('Result: {}'.format(result['sequence']))

    client.terminate()

if __name__ == '__main__':
    main()