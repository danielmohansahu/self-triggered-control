import numpy as np
from scipy.integrate import solve_ivp

class RigidBody:
    def __init__(self, initial_conditions):
        self.initial_conditions = np.array(initial_conditions)
        self.state = self.initial_conditions

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
            x3 = state[2]
            return [1, 1, 1]
        
        # solve for the response over the given time period
        ic = self.state

        # solve for the response and update our state
        self.state = solve_ivp(ode, (0, period), ic).y[:,1]

    def calculateCommand(self, state):
        """ Calculate the control for the given state.
        """
        x1 = state[0]
        x2 = state[1]
        x3 = state[2]
        return 1

    @classmethod
    def triggerCondition(cls, x1, x2, x3):
        """ Calculate the next time at which control should be executed
        """
        return 0.01

    @classmethod
    def analogResponse(cls, duration, initial_conditions, times):
        """ Solve for the "perfect world" response (i.e. non-digital)
        """
        def ode(time, state):
            x1 = state[0]
            x2 = state[1]
            x3 = state[2]
            dx1 = -x1 * x2 - 2 * x2 * x3 - x1 - x3
            dx2 = 2 * x1 * x2 * x3 + 3 * x3**2 - x2
            dx3 = x1 * x2
            return [dx1, dx2, dx3]
        
        # solve for the response and update our state
        response = solve_ivp(ode, (times[0], times[-1]), initial_conditions)

        import pdb;pdb.set_trace()

        return response.t, response.y