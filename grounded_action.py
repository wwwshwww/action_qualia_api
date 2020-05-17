import ros_mobile_client
from collections import deque
from abc import ABCMeta, abstractmethod

class GroundedActionPool():
    def __init__(self, actions: [GroundedAction]):
        self.pool = {n.action_name : n for n in actions}

    def append_action(self, actions: [GroundedAction]):
        self.pool.update({n.action_name : n for n in actions})


class GroundedAction(metaclass=ABCMeta):
    
    count = 0

    def __init__(self, name, client: ros_mobile_client.MobileClient):
        self.action_id = GroundedAction.count
        self.action_name = name
        self.mc = client
        self.step_log = deque()
        GroundedAction.count += 1

    @abstractmethod
    def _gen_candidates(self):
        raise NotImplementedError

    @abstractmethod
    def _evaluate_poses(self, candidates):
        raise NotImplementedError

    def _logging(self, candidates):
        self.step_log.append(candidates)

    def step(self):
        candi = self._gen_candidates()
        step = self._evaluate_poses(candi)
        self._logging(step)
        return step

class GroundedStep():
    def __init__(self, owner: GroundedAction, candidates: list):
        self.owner = owner
        self.candidates = candidates
        self.evaluations = [None]*len(candidates)
        self.elected_pose_id = -1
        self.elected_action_id = -1

    def adopt(self):
        pass

