import roslibpy
import numpy as np
import quaternion
import time

import ros_mobile_client as rmc
from grounded_action import GroundedAction, GroundedActionPool, GroundedStep
import area_globalization as ag
from occupancygrid_tools import AreaChecker

from typing import List, Dict, Sequence

import matplotlib.pyplot as plt
# %matplotlib inline

COLOR_OBSTACLE = 100
COLOR_UNKNOWN = -1
COLOR_PASSABLE = 0

obs_thresh = 7
unk_thresh = 2

sel_count = 3

def random_select(l: Sequence, c: int=sel_count) -> List:
    return [l.pop(np.random.randint(len(l))) for i in range(min(len(l), c))]
        
class AreaSelection(GroundedAction):
    def _gen_candidates(self) -> GroundedStep:
        obs = ag.get_obstacles(self.mc.map_data, obs_thresh, COLOR_OBSTACLE)
        filterd = list(filter(lambda o: o.convex_area>0, obs))
        sel = random_select(filterd)

        gs = [s.convex_center_of_gravity for s in sel]
        gsxy = [self.mc.get_coordinates_from_index(g) for g in gs]

        pos = [np.quaternion(0,g[0],g[1],0) for g in gsxy]
        ori = [self.mc.orientation for n in range(len(pos))]
        are = [s.convex_area for s in sel]

        return GroundedStep(self, pos=pos, ori=ori, are=are)

    def _evaluate(self, candidates: GroundedStep) -> GroundedStep:
        are = candidates.candidates['are']
        areasum = sum(are)

        for i in range(len(are)):
            ev = are[i]/areasum*np.random.random()
            candidates.evaluations[i] = ev

        candidates.selected_cand_id = np.argmax(candidates.evaluations)
        return candidates

class Observation(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate(self, candidates):
        pass

class MapExploration(GroundedAction):
    def _gen_candidates(self):
        ac = AreaChecker(self.mc.map_data, self.mc.get_index_from_coordinates(self.mc.position))
        print(np.unique(ac.color_map), ac.color_map.shape)
        unks = ag.get_obstacles(ac.color_map, unk_thresh, COLOR_UNKNOWN)
        print(f'found unknown area: {len(unks)}')
        sel = random_select(unks)

#         plt.gray()
#         ac.color_map[ac.color_map==-1] = 50
#         plt.imshow(ac.color_map)
#         plt.imshow(self.mc.map_data)

        gs = [s.convex_center_of_gravity+ac.start for s in sel]
        print(f'gs: {gs}')
        gsxy = [self.mc.get_coordinates_from_index(g) for g in gs]

        pos = [np.quaternion(0,g[0],g[1],0) for g in gsxy]
        ori = [self.mc.get_orientation_from_body(g) for g in pos]
        are = [s.convex_area for s in sel]

        return GroundedStep(self, pos=pos, ori=ori, are=are)

    def _evaluate(self, candidates):
        are = candidates.candidates['are']
        areasum = sum(are)

        for i in range(len(are)):
            ev = 0 if areasum == 0 else are[i]/areasum
            candidates.evaluations[i] = ev

        candidates.selected_cand_id = np.argmax(candidates.evaluations)
        return candidates

class GAScheduler(GroundedAction):
    def __init__(self, name, client, pool):
        super().__init__(name, client)
        self.pool = pool

    def _gen_candidates(self) -> List[GroundedStep]:
        return [a.step() for a in self.pool]

    def _evaluate(self, candidates):
        pass

def main():
    rosclient = roslibpy.Ros('10.244.1.176', port=9090)
    rosclient.on_ready(lambda: print('is ROS connected: ', rosclient.is_connected))
    rosclient.run()

    mc = rmc.MobileClient(rosclient, lambda *s: print(f'【reached】 result: {s}'))
    mc.wait_for_ready()
    print(f'pos: {mc.position}, map_shape: {mc.map_data.shape}')
    ga_map = MapExploration('map_exploration', mc)
    ga_are = AreaSelection('area_selection', mc)
    ga_obs = Observation('observation', mc)
    
    # pool_grounded = GroundedActionPool([ga_map, ga_are, ga_obs])

    # sched = GAScheduler('top_scheduler', mc, pool_grounded)

    # while(True):
    #     if mc.is_reached:
    #         pos, ori = sched.step()
    #         mc.set_goal(pos, ori)

    s = ga_map.step()
    pos = s.candidates['pos'][s.selected_cand_id]
    ori = s.candidates['ori'][s.selected_cand_id]
    mc.start()
    mc.set_goal(pos, ori)
    time.sleep(15)
    mc.stop()
    rosclient.terminate()
    print(s.candidates)

if __name__ == '__main__':
    main()