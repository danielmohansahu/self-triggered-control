import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# stub implementation of JetEngine model
class JetEngine:
    def __init__(self):
        ...
        

    def plot(self, duration = 3, initial_conditions = [5.37, 0.34]):
        """ Plot the time response of the system.
        """
        # convert initial conditions to x1,y coordinates
        ic = [initial_conditions[0], self.X2toY(initial_conditions[0], initial_conditions[1])]

        # solve for the response
        times = np.linspace(0, duration, 100)
        response = odeint(self.ode, ic, times)

        # convert result back into x1,x2 coordinates
        for resp in response:
            resp[1] = self.YtoX2(resp[0], resp[1])

        # plot the response
        plt.plot(times, response[:, 0])
        plt.plot(times, response[:, 1])
        plt.xlabel("Time (s)")
        plt.ylabel("State")
        plt.legend(["x1", "x2"])
        plt.show()

    def X2toY(self, x1, x2):
        """ Mapping from x2 to y coordinates.
        """
        return 2 * (x1**2 + x2) / (x1**2 + 1)

    def YtoX2(self, x1, y):
        """ Mapping from y to x2 coordinates.
        """
        return 0.5 * y * (x1**2 + 1) - x1**2

    def ode(self, state, time):
        """ The ODE determining the state of the system.
        """
        x1 = state[0]
        y = state[1]
        dx1 = -0.5 * (x1**2 + 1) * (x1 + y)
        dy = -(x1**2 + 1) * y
        return [dx1, dy]

