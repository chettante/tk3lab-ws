import logging
from enum import Enum
import time

logger = logging.getLogger("StateMachine")

class StateType(Enum):
    IDLE = "IDLE"
    TRACKING = "TRACKING"
    SEARCHING = "SEARCHING"

class StateMachine:
    def __init__(self, initial_state):
        self.current_state = initial_state
        self.idle_state = None
        self.tracking_state = None
        self.searching_state = None

    def update(self):
        next_state = self.current_state.update()

        if next_state == StateType.TRACKING:
            self.tracking_state.enter_time = time.time()
            self.current_state = self.tracking_state
        elif next_state == StateType.SEARCHING:
            self.searching_state.enter_time = time.time()
            self.current_state = self.searching_state
        elif next_state == StateType.IDLE:
            self.idle_state.enter_time = time.time()
            self.current_state = self.idle_state

    def get_current_state(self):
        if self.current_state is self.idle_state:
            return StateType.IDLE
        elif self.current_state is self.tracking_state:
            return StateType.TRACKING
        elif self.current_state is self.searching_state:
            return StateType.SEARCHING
        return None