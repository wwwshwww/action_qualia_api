import numpy as np
import quaternion
import math
from matplotlib import pyplot as plt

    # base_angle_q = quaternion.from_euler_angles([0,0,base_angle])
    # base_r = 30
    # base_pos = (int(base_r*math.sin(base_angle)),int(base_r*math.cos(base_angle)))
    # to_pos = (10, 0)
    # to_angle = math.atan2(to_pos[0], to_pos[1])

    # to_angle_q = quaternion.from_euler_angles([0,to_angle,0])
    
    # print(quaternion.as_euler_angles(to_angle_q*base_angle_q))
    # base_goal_rmat = quaternion.as_rotation_matrix(to_angle_q*base_angle_q)
    # r = math.sqrt(to_pos[0]**2 + to_pos[1]**2)
    # base_ahead_y = (base_pos[1], 0, base_pos[0]+r)
    # goal_pos = np.dot(base_goal_rmat, base_ahead_y)

    # print(base_angle_q**(-1))
    # print(base_angle_q.conj() / base_angle_q.abs()**2)

def main():
    mapimg = np.zeros([100,100])
    # base_angle = 0
    base_angle = math.pi/6
    base_r = 30
    base_orient = (0,0,math.pi/12)
    to_angle = math.pi/6
    to_r = 60

    base_angle_q = quaternion.from_euler_angles(0,0,base_angle)
    base_pos = (int(base_r*math.cos(base_angle)),int(base_r*math.sin(base_angle)),0)
    base_pos_q = np.quaternion(0, base_pos[0], base_pos[1], base_pos[2])
    base_orient_q = quaternion.from_euler_angles(base_orient)
    to_angle_q = quaternion.from_euler_angles(0,0,to_angle)
    to_pos = (int(to_r*math.cos(to_angle)),int(to_r*math.sin(to_angle)),0)
    to_pos_q = np.quaternion(0, to_pos[0], to_pos[1], to_pos[2])

    # ijk = np.array([0,0,1])*math.sin(to_angle/2)
    # to_angle_q = np.quaternion(math.cos(to_angle/2), ijk[0], ijk[1], ijk[2])
    print(base_pos)

    base_dq_t = [base_angle_q, 0.5*base_pos_q*base_angle_q]
    base_dq_vec = [1, base_pos_q]
    to_dq = [to_angle_q, 0.5*to_pos_q*to_angle_q]
    to_dq_conj1 = [to_dq[0], (-1)*to_dq[1]]
    to_dq_conj2 = [to_dq[0].conj(), to_dq[1].conj()]
    to_dq_conj3 = [to_dq_conj2[0], (-1)*to_dq_conj2[1]]
    # print(to_dq[0]*to_dq[0], to_dq[1]*to_dq[0]-to_dq[0]*to_dq[1])

    ## memo (ref: https://qiita.com/mebiusbox2/items/2fa0f0a9ca1cf2044e82)
    # r: 相対的に回転させたい量（姿勢）を表すクォータニオン
    # v: 移動対象のベクトルを表す純クォータニオン
    # r': rの逆クォータニオン
    # t: 相対的に平行移動させたい量（ベクトル）を表す純クオータニオン
    # transrated_pose_dq = 1 + ε(rvr'+t)
    
    goal_pos_dq = [1, to_dq[0]*base_pos_q*to_dq[0].conj()+to_pos_q]
    goal_pos_q = to_angle_q*base_pos_q*to_angle_q.conj()
    goal_pos_orient_dq = [1, base_orient_q*goal_pos_dq[1]*base_orient_q.conj()]

    mapimg[int(goal_pos_q.x), int(goal_pos_q.y)] = 255
    mapimg[int(goal_pos_dq[1].x), int(goal_pos_dq[1].y)] = 255
    mapimg[int(goal_pos_orient_dq[1].x), int(goal_pos_orient_dq[1].y)] = 255
    mapimg[base_pos[0], base_pos[1]] = 255

    # orient = np.array((4,5,0))
    # orient = orient / np.linalg.norm(orient)
    # print(orient)
    # orient_q = quaternion.from_rotation_vector(orient)
    # print(orient_q)

    plt.gray()
    plt.imshow(mapimg)
    plt.show()

if __name__ == "__main__":
    main()