import time
import roslibpy as rlp

def main():
    client = rlp.Ros('10.244.1.117', port=9090)
    client.run()
    print('con: ', client.is_connected)

    # meta_listener = rlp.Topic(client, 'nav_msgs/MapMetaData', )
    # map_lis = rlp.Topic(client, 'nav_msgs/OccupancyGrid', )

    # map_service = rlp.Service(client, 'nav_msgs/GetMap', 'nav_msgs/OccupancyGrid')
    # map_req = rlp.ServiceRequest()

    # print('Calling GetMap...')
    # result = map_service.call(map_rls
    # eq)
    # print(f'map: {result}')

    map_listener = rlp.Topic(client, '/map', 'nav_msgs/OccupancyGrid')
    map_listener.subscribe(lambda mess: print('s', mess))

    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()

if __name__ == "__main__":
    main()