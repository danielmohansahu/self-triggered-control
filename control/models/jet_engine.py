import numpy as np
from scipy.integrate import solve_ivp

# stub implementation of JetEngine model
class JetEngine:
    tau_trigger = 0.00763
    tau_periodic = 0.00763

    def __init__(self, initial_conditions, noise_stddev=0.0):
        self.initial_conditions = np.array(initial_conditions)
        self.state = self.initial_conditions

        # the following _should_ just cancel out
        self.beta = 1

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
            x1 = state[0]
            x2 = state[1]
            dx1 = -x2 - 1.5 * x1**2 - 0.5 * x1**3
            dx2 = (x1 - command) / (self.beta**2)
            return [dx1, dx2]
        
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
        y = self.X2toY(x1, x2)
        # return x1 - 0.5 * (x1**2 + 1) * (y + y * x1**2 + x1 * y**2) * self.beta**2 + 2 * x1 * self.beta**2
        # re-derived; the above appears to be incorrect in the paper
        return x1 + 0.5 * self.beta**2 * (x1**2 + 1) * (2 * x1**2 * y + y + x1 * y**2 - 2 * x1**2 - 2 * x1 * y)

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

    @classmethod
    def periodicCondition(cls, state):
        """ Calculate the next time at which control should be executed
        """
        return cls.tau_periodic

    @classmethod
    def triggerCondition(cls, state):
        """ Calculate the next time at which control should be executed
        """
        x1, x2 = state
        y = cls.X2toY(x1, x2)
        norm = np.math.sqrt(x1**2 + y**2)

        # calculate next trigger step
        dt = (29 * x1 + norm**2) / (5.36 * norm * x1**2 + norm**2) * cls.tau_trigger
        return dt

    @classmethod
    def analogResponse(cls, duration, initial_conditions, times):
        """ Solve for the "perfect world" response (i.e. non-digital)
        """
        def ode(time, state):
            x1 = state[0]
            y = state[1]
            dx1 = -0.5 * (x1**2 + 1) * (x1 + y)
            dy = -(x1**2 + 1) * y
            return [dx1, dy]
        
        # solve for the response over the given time period
        ic = np.array([initial_conditions[0], cls.X2toY(initial_conditions[0], initial_conditions[1])])

        # solve for the response and update our state
        response = solve_ivp(ode, (times[0], times[-1]), ic, t_eval=times)

        # convert back to x1, x2 coordinates
        state = None
        for i in range(0, response.y.shape[1]):
            x1 = response.y[:,i][0]
            y = response.y[:,i][1]
            x2 = cls.YtoX2(x1, y)

            if state is None:
                state = np.array([x1, x2])
            else:
                state = np.vstack([state, np.array([x1, x2])])

        return response.t, state