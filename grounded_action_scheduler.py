import mobile_client
from abc import ABCMeta, abstractmethod

class GroundedActionFactory(ABCMeta):

    @abstractmethod
    def _select_step(self, owner):
        pass

    @abstractmethod
    def _logging_step(self, step):
        pass

    def select(self, owner):
        self.__s = self._select_step(owner)
        self._logging_step(self.__s)
        return self.__s

class GroundedStep(ABCMeta):

    @abstractmethod
    def evaluate(self):
        pass


class GroundedActionPool():
    pass