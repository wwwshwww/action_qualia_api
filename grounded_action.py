import ros_mobile_client
from collections import deque
import numpy as np
import quaternion

from abc import ABCMeta, abstractmethod
from typing import List, Dict

class GroundedActionPool():
    def __init__(self, actions: List[GroundedAction]):
        self.pool = {n.action_name : n for n in actions}

    def append_action(self, actions: List[GroundedAction]):
        self.pool.update({n.action_name : n for n in actions})

class GroundedStep():
    ## candidates: Dict['pos': List[np.quaternion], 'ori': List[np.quaternion]]
    def __init__(self, owner: GroundedAction, **candidates: List):
        self.owner = owner
        self.candidates: Dict[str:List] = {c: candidates[c] for c in candidates}
        self.evaluations: List[float] = [None]*len(candidates['pos'])
        self.selected_pose_id = -1
        self.selected_action_id = -1

    def adopt(self):
        pass

class GroundedAction(metaclass=ABCMeta):
    
    count = 0

    def __init__(self, name, client: ros_mobile_client.MobileClient):
        self.action_id = GroundedAction.count
        self.action_name = name
        self.mc = client
        self.step_log = deque()
        GroundedAction.count += 1

    @abstractmethod
    def _gen_candidates(self) -> GroundedStep:
        raise NotImplementedError

    @abstractmethod
    def _evaluate(self, candidates: GroundedStep):
        raise NotImplementedError

    def _logging(self, step: GroundedStep):
        self.step_log.append(step)

    def step(self):
        candi = self._gen_candidates()
        step = self._evaluate(candi)
        self._logging(step)
        return step

