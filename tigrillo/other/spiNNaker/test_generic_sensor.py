"""
Test example to validate a generic analog sensor on the Tigrillo robot
"""

import spynnaker8 as p

from generic_sensor import AnalogDeviceDataHolder
from pyNN.utility.plotting import Figure, Panel
import matplotlib.pyplot as plt


print "\n== 1. Set-up the simulation =="
p.setup(timestep=1.0, min_delay=1.0, max_delay=32.0)
t_stop = 5000
sensor_population_size = 20
control_pop_size = 20

print "\n== 2. Create the populations =="
input_population = p.Population(sensor_population_size, p.SpikeSourcePoisson, {'rate': 10}, "poisson")
sensor_population = p.Population(sensor_population_size, AnalogDeviceDataHolder(spinnaker_link_id=1), label="sens")
control_population = p.Population(control_pop_size, p.IF_curr_exp, cellparams={}, label="if_curr")

print "\n== 3. Create the connections =="
p.Projection(sensor_population, control_population,  p.AllToAllConnector(), receptor_type='excitatory',
             synapse_type=p.StaticSynapse(weight=1.0))

print "\n== 4. Run the simulation and record the spikes =="
control_population.record("spikes")
p.run(t_stop)

print "\n== 5. Display the results =="
control_spikes = control_population.get_data("spikes")
Figure(
    Panel(control_spikes.segments[0].spiketrains, xlabel="Time/ms", xticks=True,
          yticks=True, markersize=2, xlim=(0, t_stop)),
    title="spikes",
    annotations="Simulated with {}".format(p.name())
)
plt.show()
p.end()

