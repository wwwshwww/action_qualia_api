import ros_mobile_client
from grounded_action import GroundedAction, GroundedActionPool, GroundedStep

class MapExploration(GroundedAction):
    pass
    

class GAScheduler(GroundedAction):
    def __init__(self, name, client, pool):
        super().__init__(name, client)
        self.pool = pool


