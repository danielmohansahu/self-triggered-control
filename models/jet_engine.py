import numpy as np
from scipy.integrate import solve_ivp

# stub implementation of JetEngine model
class JetEngine:
    def __init__(self, initial_conditions):
        self.initial_conditions = np.array(initial_conditions)
        self.state = self.initial_conditions
        # the following _should_ just cancel out
        self.beta = 1

    def reset(self):
        """ Clear current state
        """
        self.state = self.initial_conditions

    def getState(self):
        """ Return the current model state.
        """
        return self.state

    def applyCommand(self, command, period):
        """ Determine our response to the given command over the given period
        """
        # update our closed loop response with the given command
        def ode(time, state):
            x1 = state[0]
            x2 = state[1]
            dx1 = -x2 - 1.5 * x1**2 - 0.5 * x1**3
            dx2 = (x1 - command) / (self.beta**2)
            return [dx1, dx2]
        
        # solve for the response over the given time period
        ic = self.state

        # solve for the response and update our state
        self.state = solve_ivp(ode, (0, period), ic).y[:,1]

    def calculateCommand(self, state):
        """ Calculate the control for the given state.
        """
        x1 = state[0]
        x2 = state[1]
        y = self.X2toY(x1, x2)
        return x1 - 0.5 * (x1**2 + 1) * (y + y * x1**2 + x1 * y**2) * self.beta**2 + 2 * x1 * self.beta**2

    @staticmethod
    def X2toY(x1, x2):
        """ Mapping from x2 to y coordinates.
        """
        return 2 * (x1**2 + x2) / (x1**2 + 1)

    @staticmethod
    def YtoX2(x1, y):
        """ Mapping from y to x2 coordinates.
        """
        return 0.5 * y * (x1**2 + 1) - x1**2

