import roslibpy as rlp
import roslibpy.actionlib
import numpy as np
import math
import quaternion # numpy-quaternion
from collections import deque
import time
from threading import Timer

from ros_client import RosClient

## FCFS (First Come First Served）
class ActionScheduler(Timer):
    STATUS_READY_TO_START = 0b01
    def __init__(self, ros_client: RosClient, server_name: str, action_name: str, callback: callable, queue_size: int=20, rate: float=0.01, args: list=None, kwargs: dict=None): 
        Timer.__init__(self, rate, self.run, args, kwargs)
        self.ros_client = ros_client
        self.action_client = rlp.actionlib.ActionClient(ros_client.client, server_name, action_name)
        self.current_goal: rlp.actionlib.Goal = None
        self.callback = callback
        self.thread = None
        self.goal_queue = deque(maxlen=queue_size)
        self.rate = rate
        self.state = ActionScheduler.STATUS_READY_TO_START ## 01: queue is empty, 10: task is actived
        
    def _update_state(self):
        is_emp = 1 if len(self.goal_queue) == 0 else 0
        is_fin = 0
        if self.current_goal is not None:
            is_fin = 0 if self.current_goal.is_finished else 1
        self.state = is_emp|(is_fin<<1)

    def _check_task(self):
        self._update_state()
        self.thread = Timer(self.rate, self._check_task)
        self.thread.start()

        if not self.state&1 and not self.state>>1&1:
            self.current_goal = self.goal_queue.pop()
            self._update_state()
            self.current_goal.send(self._finish)

    def _finish(self, result):
        self._update_state()
        self.callback(result)

    def append_goal(self, goal: rlp.actionlib.Goal):
        self.goal_queue.append(goal)
        self._update_state()

    def run(self):
        self._check_task()

    def cancel(self):
        if self.thread is not None:
            self.thread.cancel()
            self.thread.join()
            del self.thread