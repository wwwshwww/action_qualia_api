import ros_mobile_client
from grounded_action import GroundedAction, GroundedActionPool, GroundedStep

class MapExploration(GroundedAction):
    def _gen_candidates(self):
        pass

    def _evaluate_poses(self, candidates):
        pass
    
class ElectErea(GroundedAction):
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

