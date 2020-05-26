import os
from state import State
import rospy
from std_msgs.msg import String


class MarkovDP(object):
    def __init__(self):
        self.states = {}
        self.get_states()
        self.get_transitions()
        self.inRoutine = False

        self.current_state = self.states[list(self.states.keys())[0]]  # Stopped state

    def get_states(self):
        with open("transitions.csv", "r") as f:
            for line in f:
                name = line.split(',')[0]
                state = State(name)
                if name not in self.states:
                    self.states[name] = state
        print(str(self.states))

    def get_transitions(self):
        with open("transitions.csv", "r") as f:
            for line in f:
                name, action, next_state = line.strip().split(', ')
                self.states[name].transisitons.append((action, next_state))

    def iterate(self):
        if self.current_state == "stopped":
            self.stopped_routine()
        if self.current_state == "follow_lanes":
            self.follow_lanes_routine()
        # Is another routine running?
        # Interruptions?
        # Check Lane Detection
        #       Adjust lanes
        # Check Object Detection
        #       Stop Sign Detected
        #           Run Stop Sign Routine
        #       Red Stop Light Detected
        #           Run Stop Sign Routine
        # Run Routines
        self.current_state = self.states[list(self.states.keys())[1]]

    def stopped_routine(self):
        print("Running stopped routine")
        # Check for lanes
        lane_msg = rospy.wait_for_message("LaneDetection", String, timeout=None)
        # Once found switch to follow lane state
        # TODO add lanes not found to Lane Detection
        if lane_msg != "lanes not found":
            self.current_state = "follow_lanes"

    def follow_lanes_routine(self):
        print("Running follow lanes routine")
        # Follow Lanes
        # Adjust to stay in lanes


if __name__ == "__main__":
    mdp = MarkovDP()
    print(mdp.current_state)
    mdp.iterate()
    print(mdp.current_state)
