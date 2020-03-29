import roslibpy as rlp

def main():
    client = rlp.Ros('10.244.1.117', port=9090)
    client.run()
    print('con: ', client.is_connected)

    listener = rlp.Topic(client, '/odom', 'nav_msgs/Odometry')
    listener.subscribe(lambda message: print(message))
    client.terminate()
    

if __name__ == '__main__':
    main()