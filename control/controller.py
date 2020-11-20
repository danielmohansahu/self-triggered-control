""" A simple periodic controller that executes a given controller on a given plant periodically.
"""
import numpy as np

class Controller:
    def __init__(self, model, control):
        # the plant model to execute
        self.model = model

        # the control law to apply
        self.control = control

    def executePeriodic(self, duration, period):
        """ Apply the periodic control law to the given model.
        """
        # initialize loop variables
        self.model.reset()
        t = 0
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
            self.model.applyCommand(u, period)

            # increment time
            t += period

        return (times, response)

    def executeTrigger(self, duration, trigger_condition):
        """ Apply self-triggered control law to the given model.
        """
        # initialize loop variables
        self.model.reset()
        t = 0
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

            # get next trigger
            dt = trigger_condition(state)

            self.model.applyCommand(u, dt)

            # increment time
            t += dt

        return (times, response)