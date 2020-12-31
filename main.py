import os
import sys
import optparse

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

#from sumolib import checkBinary  # noqa
import traci  # noqa

def run(options=None):
    """execute the TraCI control loop"""
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print(traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity'))
        step += 1

    traci.close()
    sys.stdout.flush()

# Adds additional info to end of
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="Run the commandline version of sumo")
    optParser.add_option("--algorithm", action="store_true",
                         default=False, help="Run algorithm on simulation")
    options, args = optParser.parse_args()
    return options
