import numpy as np
import quaternion
import math
from matplotlib import pyplot as plt

def get_base_pose_from(base_vec: np.quaternion, base_orient: np.quaternion, rel_vec: np.quaternion, rel_orient: np.quaternion) -> (np.quaternion, np.quaternion):
    ## rotate angle (alpha,beta,gamma):(atan(z/y),atan(x/z),atan(x/y))
    to_angle = (math.atan2(rel_vec.z, rel_vec.y), math.atan2(rel_vec.x, rel_vec.z), math.atan2(rel_vec.x,rel_vec.y))
    to_angle_q = quaternion.from_euler_angles(to_angle)
    t = (-base_orient) * rel_vec * (-base_orient).conj()
    goal_vec = base_vec+t
    goal_orient = base_orient*rel_orient
    return goal_vec, goal_orient

def main():
    mapimg = np.zeros([100,100])
    # base_angle = 0
    base_angle = math.pi/6
    base_r = 30
    to_angle = math.pi/6
    to_r = 60

    base_pos = (base_r*math.cos(base_angle), base_r*math.sin(base_angle), 0)
    to_pos = (to_r*math.cos(to_angle), to_r*math.sin(to_angle), 0)
    print(base_pos)

    g_pos = (base_pos[0]+to_r*math.cos(base_angle+to_angle),base_pos[1]+to_r*math.sin(base_angle+to_angle))
    print(g_pos)
    mapimg[int(base_pos[0]), int(base_pos[1])] = 255
    mapimg[int(g_pos[0]), int(g_pos[1])] = 255

    base_pos_q = np.quaternion(0, *base_pos)
    to_pos_q = np.quaternion(0, *to_pos)
    base_angle_q = quaternion.from_euler_angles(0,0,base_angle)
    to_angle_q = quaternion.from_euler_angles(0,0,to_angle)
    rotated_pos = to_angle_q*base_pos_q*to_angle_q.conj()
    mapimg[int(rotated_pos.x), int(rotated_pos.y)] = 255

    theta = math.atan2(to_pos[1], base_r+to_pos[0])
    theta_q = quaternion.from_euler_angles(0,0,theta)
    gg = theta_q*base_pos_q*theta_q.conj()
    mapimg[int(gg.x), int(gg.y)] = 255

    t = (-base_angle_q)*to_pos_q*((-base_angle_q).conj())
    print(t)

    r = theta_q
    sigma = [r, 0.5*t*r]
    sigma_conj1 = [sigma[0], -sigma[1]]
    sigma_conj2 = [sigma[0].conj(), sigma[1].conj()]
    sigma_conj3 = [sigma_conj2[0], -sigma_conj2[1]]

    liner = base_pos_q+t
    print(liner)
    mapimg[int(liner.x), int(liner.y)] = 255

    ggg = [1, r*(base_pos_q+t)*r.conj()]

    print(get_base_pose_from(base_pos_q, base_angle_q, to_pos_q, to_angle_q))

    plt.gray()
    plt.imshow(mapimg)
    plt.show()

if __name__ == '__main__':
    main()