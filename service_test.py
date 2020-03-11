import roslibpy as rlp
import matplotlib.pyplot as plt
import numpy as np

def main():
    client = rlp.Ros('10.244.1.117', port=9090)
    client.run()
    print('con: ', client.is_connected)

    service = rlp.Service(client, '/dynamic_map', '/nav_msgs/GetMap')
    request = rlp.ServiceRequest()

    result = service.call(request)
    print(type(result), result['map'].keys())
    
    print(result['map']['info'])
    print(result['map']['header'])
    print(len(result['map']['data']))

    width = result['map']['info']['width']
    height = result['map']['info']['height']
    ar = np.array(result['map']['data']).reshape([width,height])
    
    ar = np.where(ar == -1, 0, ar)

    plt.gray()
    plt.imshow(ar)
    plt.show()
    

    client.terminate()



    # rlp.ServiceResponse.

if __name__ == "__main__":
    main()