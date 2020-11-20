""" A simple periodic controller that executes a given controller on a given plant periodically.
"""
import numpy as np

class Controller:
    def __init__(self, model, control):
        # the plant model to execute
        self.model = model

        # the control law to apply
        self.control = control

    def execute(self, duration, trigger_condition, disturbance=None):
        """ Apply self-triggered control law to the given model.
        """
        # initialize loop variables
        self.model.reset()
        t = 0
        times = []
        response = None

        # check if we're supposed to have a disturbance
        if disturbance is not None and len(disturbance) != 2:
            raise RuntimeError("Disturbance must be of form [time, (s1, s2, ...)]")

        # execute until duration is over
        while (t <= duration):
            # get current model state
            state = self.model.getState()

            # save response
            times.append(t)
            if response is None:
                response = state
            else:
                response = np.vstack([response, state])

            # calculate control for this state
            u = self.control(state)

            # get next trigger
            dt = trigger_condition(state)

            if disturbance is not None and disturbance[0] < t:
                self.model.applyCommand(u, dt, disturbance[1])
                disturbance = None
            else:
                self.model.applyCommand(u, dt)

            # increment time
            t += dt

        return (times, response)