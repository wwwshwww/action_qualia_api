import mobile_client
from collections import deque
from abc import ABCMeta, abstractmethod

class GroundedActionPool():
    def __init__(self, actions: {str: callable}):
        self.pool = {n: actions[n](n) for n in actions}

    def append_action(self, name, instance_lambda):
        self.pool[name] = instance_lambda()

class GroundedAction(metaclass=ABCMeta):
    
    count = 0

    def __init__(self, name):
        self.action_id = GroundedAction.count
        self.action_name = name
        self.step_log = deque()
        GroundedAction.count += 1

    @abstractmethod
    def _gen_candidates(self):
        pass

    @abstractmethod
    def _evaluate_poses(self, candidates):
        pass

    @abstractmethod
    def _log_candidates(self, candidates):
        self.step_log.append(candidates)

    def step(self):
        self.__candi = self._gen_candidates()
        return self._evaluate_poses(self.__candi)

class GroundedStep(metaclass=ABCMeta):

    @abstractmethod
    def adopt(self):
        pass
