""" A simple periodic controller that executes a given controller on a given plant periodically.
"""
import numpy as np

class PeriodicController:
    def __init__(self, period, model, control):
        # the time period at which to execute 
        self.period = period

        # the plant model to execute
        self.model = model

        # the control law to apply
        self.control = control

    def execute(self, duration):
        """ Apply the periodic control law to the given model.
        """
        # initialize time
        t = 0

        # reset model
        self.model.reset()

        # initialize return variables (response)
        times = []
        response = None

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
            self.model.applyCommand(u, self.period)

            # increment time
            t += self.period

        return (times, response)