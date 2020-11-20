import numpy as np
from scipy.integrate import solve_ivp

class RigidBody:
    tau_periodic = 4.5 * 10e-5
    tau_trigger = 0.0051

    def __init__(self, initial_conditions, noise_stddev=0):
        self.initial_conditions = np.array(initial_conditions)
        self.state = self.initial_conditions

        # if desired, inject noise into the response
        self.noise = lambda : np.random.normal(0, noise_stddev)

    def reset(self):
        """ Clear current state
        """
        self.state = self.initial_conditions

    def getState(self):
        """ Return the current model state.
        """
        return self.state

    def applyCommand(self, command, period, disturbance=None):
        """ Determine our response to the given command over the given period
        """
        # update our closed loop response with the given command
        def ode(time, state):
            x1, x2, _ = state
            u1, u2 = command
            return [u1, u2, x1*x2]
        
        # solve for the response over the given time period
        ic = self.state

        # solve for the response and update our state
        self.state = solve_ivp(ode, (0, period), ic).y[:,1]

        # add noise
        self.state = np.array([s + self.noise() for s in self.state])

        # add disturbance (perturbation)
        if disturbance is not None:
            self.state += disturbance
        
    def calculateCommand(self, state):
        """ Calculate the control for the given state.
        """
        x1 = state[0]
        x2 = state[1]
        x3 = state[2]
        u1 = -x1 * x2 - 2 * x2 * x3 - x1 - x3
        u2 = 2 * x1 * x2 * x3 + 3 * x3**2 - x2
        return [u1, u2]

    @classmethod
    def periodicCondition(cls, state):
        """ Calculate the next time at which control should be executed
        """
        return cls.tau_periodic

    @classmethod
    def triggerCondition(cls, state):
        """ Calculate the next time at which control should be executed
        """
        return cls.tau_trigger / (1 + np.linalg.norm(state) ** 2)

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
        response = solve_ivp(ode, (times[0], times[-1]), initial_conditions, t_eval=times)
        return response.t, response.y