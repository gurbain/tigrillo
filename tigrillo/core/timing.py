"""
Provide tools for timing real-time and faster than real-time simulations
"""

from copy import copy
import datetime
import pause

__author__ = "Gabriel Urbain"
__copyright__ = "Copyright 2017, Human Brain Project, SP10"

__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be"
__status__ = "Research"
__date__ = "September 19th, 2017"


class Timer(object):

    def __init__(self, real_time=True, runtime=10, dt=0.01, print_dt=0.5):

        self.real_time = real_time
        self.sdt = dt
        self.rdt = datetime.timedelta(seconds=dt)
        self.rt = None
        self.st = 0

        self.st_init = 0
        self.st_end = runtime
        self.rt_init = None
        self.rt_end = None

        self.it = 0
        self.n_it = int(self.st_end / self.sdt)

        self.print_dt = print_dt
        self.time_since_print = 0

    def start(self):
        """ Start the real-time clock """

        self.rt = datetime.datetime.now()
        self.rt_init = copy(self.rt)
        self.rt_end = self.rt_init + datetime.timedelta(seconds=self.st_end)

        self.st = 0
        self.st_init = 0
        self.print_info()

    def update(self):
        """ Update time for one iteration"""

        # Update the times
        new_rt = datetime.datetime.now()
        new_st = self.st + self.sdt
        self.rdt = new_rt - self.rt

        # Print info with a given frequency
        if self.time_since_print < self.print_dt:
            self.time_since_print += self.sdt
        else:
            self.time_since_print = 0
            self.print_info()

        # If the real time is too slow for the simulation time
        if (new_rt - self.rt_init).total_seconds() > (new_st - self.st_init):
            print('Warning: the real time step (' + str(self.rdt)
                  + ') is higher than the desired one (' + str(self.sdt) +
                  ')! Please decrease computation load (IO reading, controller,...) or increase the desired timestep')
            self.st += self.rdt.total_seconds()
            self.rt = new_rt

        # If the real time is too fast for the simulation time
        if (new_rt - self.rt_init).total_seconds() < (new_st - self.st_init):
            self.st += self.sdt
            self.rt = self.rt + datetime.timedelta(seconds=self.sdt)
            pause.until(self.rt)

        # If by chance they correspond
        else:
            self.st = new_st
            self.rt = new_rt

        self.it += 1

    def print_info(self):
        """ Print info over timing and iterations number """

        print("Epoch: " + str(self.it) + "/" + str(self.n_it) +
              ";\tSimulated time: " + "%.2f" % self.st + "/" +  "%.2f" % self.st_end +
              ";\tReal time: " + str(self.rt.strftime("%S.%f")[:-2]) + "/" + str(self.rt_end.strftime("%S.%f")[:-2]) +
              ";  \tSim dt: " + str(self.sdt) + ";\tReal dt: " + str(self.rdt.total_seconds()))

    def is_finished(self):
        """ Return a boolean set to True if the simulation is timer is over """

        return (self.st > self.st_end) or (self.rt > self.rt_end)
