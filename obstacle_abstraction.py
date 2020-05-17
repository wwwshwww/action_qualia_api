import numpy as np
import collections
import math

from typing import List

def get_test_img():
    im = np.zeros([500, 500], dtype=int)
    im[2,2] = 255
    im[4,35] = 255
    im[42,7] = 255
    im[48, 40] = 255
    im[25,1] = 255
    im[22,26] = 255
    im[23,25] = 255
    im[24,24] = 255
    im[29,29] = 255

    im[455,455] = 255
    im[451,456] = 255
    return im

class PointCluster():
    def __init__(self, img):
        self.img = img

    def get_clusters(self, size, target, erea_start=0, erea_end=None):
        if erea_end is None: 
            erea_end = len(self.img)-1
        # points = np.where(im>=255)
        # points = np.where(self.img<50)
        points = np.where(self.img==target)
        used = np.full_like(self.img, False)
        clusters = collections.deque()

        for (i, j) in zip(points[0], points[1]):
            ##print(used[i,j], (i,j))
            if used[i,j]: continue
            ##print(f"\n===={i,j}====\n")
            tmp = collections.deque()
            clust = collections.deque()
            tmp.append((i, j))
            clust.append((i, j))
            used[i,j] = True
            while not len(tmp)==0:
                ##print(tmp)
                pt = tmp.pop()
                si = max(erea_start, pt[0]-size)
                ei = min(erea_end, pt[0]+size)
                sj = max(erea_start, pt[1]-size)
                ej = min(erea_end, pt[1]+size)
                # erea = self.img[si:ei+1, sj:ej+1]
                # print((i,j),erea)
                grid_i, grid_j = np.meshgrid(range(si,ei+1), range(sj,ej+1), indexing='ij')
                ##print(grid_i, grid_j)
                dis_i = (grid_i-pt[0])**2
                ##print(dis_i)
                dis_j = (grid_j-pt[1])**2
                ##print(dis_j)
                dis_filter = np.sqrt(dis_i+dis_j)<=size
                ### f = np.where(np.logical_and(self.img[si:ei+1, sj:ej+1]>=255, dis_filter))
                f = np.where(np.logical_and(self.img[si:ei+1, sj:ej+1]==target, dis_filter))
                ##print(f)

                for (ii, jj) in zip(f[0], f[1]):
                    tmp_p = (grid_i[ii,jj], grid_j[jj,jj])
                    ##print(tmp_p)
                    if used[tmp_p]: continue
                    used[tmp_p] = True
                    ##print(tmp_p,"2")
                    tmp.append(tmp_p)
                    clust.append(tmp_p)
            clusters.append(clust)

        return clusters

class ObstacleMap():
    def __init__(self, map_img):
        self.map_img = map_img

    def trim(self, start: tuple, end: tuple) -> np.ndarray:
        return self.map_img[start[0]:end[0]+1, start[1]:end[1]+1]

class Obstacle():
    def __init__(self, ob_id, super_map: ObstacleMap, ob_threshold=0, points=[]):
        self.ob_id = ob_id
        self.super_map = super_map
        self.ob_threshhold = ob_threshold
        self.points = points ## have coordinates of xy
        self.convex_points = get_convex_points(points)
        self.minimap_start, self.minimap_end, self.minimap = self.minimapping()
        self.child: List[int] = None ## have ob_id
        self.child_threshold = None

    @property
    def have_child(self):
        return self.child is not None

    def minimapping(self) -> (int, int, np.ndarray):
        lp = np.zeros_like(self.points)
        for i, p in enumerate(self.points):
            lp[i] = [p[0], p[1]]

        max_i = np.max(lp[:,0])
        min_i = np.min(lp[:,0])
        max_j = np.max(lp[:,1])
        min_j = np.min(lp[:,1])

        pt_min = (min_i,min_j)
        pt_max = (max_i,max_j)

        return pt_min, pt_max, self.super_map.trim(pt_min, pt_max)

    def set_child(self, threshhold, target):
        self.child_threshold = threshhold
        self.child = get_obstacles(self.minimap, threshhold, target)
    
def get_convex_points(points) -> np.ndarray:
    if len(points) <= 2:
        return points
    angl = np.zeros([len(points)])
    angl[0] = -999
    for i in range(1, len(points)):
        angl[i] = get_declination(points[0], points[i])
        index = np.argsort(angl)

    vec = lambda m, s: [points[s][0]-points[m][0], points[s][1]-points[m][1]]

    hull = collections.deque()
    hull.append(index[0])
    hull.append(index[1])

    for i in range(2, len(index)):
        while True:
            tmpi = hull.pop()
            last = hull[-1]
            tcross = np.cross(vec(last,tmpi), vec(tmpi,index[i]))
            #print(tmpi, index[i], tcross)
            if tcross >= 0:
                hull.append(tmpi)
                hull.append(index[i])
                break

    convex_pts = np.zeros([len(hull), 2], dtype=int)
    for i, h in enumerate(hull):
        convex_pts[i] = (points[h][1], points[h][0])
    #print(convex_pts)
    return convex_pts
    
def get_declination(p1: tuple, p2: tuple) -> float:
    return math.atan2(p2[1]-p1[1], p2[0]-p1[0])

def get_obstacles(img, threshhold, target) -> List[Obstacle]:
    pc = PointCluster(img)
    om = ObstacleMap(img)
    clusters = pc.get_clusters(threshhold, target)
    # ob_idとかちゃんとセットしよう
    obs = [None]*len(clusters)
    for i, o in enumerate(clusters):
        obs[i] = Obstacle(ob_id=i, super_map=om, ob_threshold=threshhold, points=o)
    return obs

def main():

    img = get_test_img()
    print(img)
    obs = get_obstacles(img, 4, 255)
    for o in obs:
        print(o.minimap)

if __name__ == "__main__":
    main()
