import roslibpy
import ros_mobile_client
from grounded_action import GroundedAction, GroundedActionPool, GroundedStep

class MapExploration(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate_poses(self, candidates):
        pass
    
class EreaElection(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate_poses(self, candidates):
        pass

class Observation(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate_poses(self, candidates):
        pass

class GAScheduler(GroundedAction):
    def __init__(self, name, client, pool):
        super().__init__(name, client)
        self.pool = pool

    def _gen_candidates(self):
        pass

    def _evaluate_poses(self, candidates):
        pass

def main():
    rosclient = roslibpy.Ros('10.244.1.176', port=9090)
    rosclient.on_ready(lambda: print('is ROS connected: ', rosclient.is_connected))
    rosclient.run()

    mc = ros_mobile_client.MobileClient(rosclient, lambda: print('goal'))
    ga_map = MapExploration('map_exploration', mc)
    ga_ele = EreaElection('erea_election', mc)
    ga_obs = Observation('observation', mc)
    
    pool_grounded = GroundedActionPool([ga_map, ga_ele, ga_obs])

    sched = GAScheduler('top_scheduler', mc, pool_grounded)

if __name__ == '__main__':
    main()