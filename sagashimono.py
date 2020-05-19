import roslibpy
import numpy as np
import quaternion

import ros_mobile_client as rmc
from grounded_action import GroundedAction, GroundedActionPool, GroundedStep
import obstacle_abstraction as oa

from typing import List, Dict

abstraction_thresh = 7
ob_color = 0
        
class AreaSelection(GroundedAction):
    sel_count = 3
    def _gen_candidates(self) -> GroundedStep:
        obs = oa.get_obstacles(self.mc.map_data, abstraction_thresh, ob_color)
        filterd = list(filter(lambda o: o.convex_erea>0, obs))
        sel = [filterd.pop(np.random.randint(len(filterd))) for i in range(min(len(filterd), self.sel_count))]
        gs = [s.convex_center_of_gravity for s in sel]
        gsxy = [self.mc.get_coordinates_from_map(g) for g in gs]
        pos = [np.quaternion(0,g[0],g[1],0) for g in gsxy]
        ori = [np.quaternion(1,0,0,0) for n in range(len(pos))]
        are = [s.convex_erea for s in sel]
        return GroundedStep(self, pos=pos, ori=ori, are=are)

    def _evaluate(self, candidates: GroundedStep) -> GroundedStep:
        are = candidates.candidates['are']
        areasum = sum(are)
        for i in range(len(are)):
            ev = are[i]/areasum*np.random.random()
            candidates.evaluations[i] = ev
        return candidates

class Observation(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate(self, candidates):
        pass

class MapExploration(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate(self, candidates):
        pass

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

    mc = rmc.MobileClient(rosclient, lambda: print('goal'))
    mc.wait_for_ready()
    ga_map = MapExploration('map_exploration', mc)
    ga_are = AreaSelection('area_selection', mc)
    ga_obs = Observation('observation', mc)
    
    pool_grounded = GroundedActionPool([ga_map, ga_are, ga_obs])

    sched = GAScheduler('top_scheduler', mc, pool_grounded)

    while(True):
        if mc.is_reached:
            pos, ori = sched.step()
            mc.set_goal(pos, ori)

if __name__ == '__main__':
    main()