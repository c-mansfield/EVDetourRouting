import os
import sys
import optparse
import random
from algorithm.reroute import rerouter
from algorithm.Graph import Graph

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

#from sumolib import checkBinary  # noqa
import traci  # noqa
import sumolib

def run(netFile, additionalFile, options=None):
    """execute the TraCI control loop"""
    step = 0

    net = sumolib.net.readNet(netFile)
    graph = Graph(net, additionalFile)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if step == 100:
            add_ev(graph)

        # Checks EV battery capacity
        # print('Battery Capacity: ', traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity'))
        # print('Range Left: ', round((float(traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity')) / 330) * 1000, 2))
        # print('Energy Consumption: ', traci.vehicle.getElectricityConsumption('EV1'))
        # if traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity') == "0.00":
        #     print('Vehicle battery empty')

        step += 1

    traci.close()
    sys.stdout.flush()

# Get run parameters
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="Run the commandline version of sumo")
    optParser.add_option("--algorithm", action="store_true",
                         default=False, help="Run algorithm on simulation")
    options, args = optParser.parse_args()
    return options

# Adds electric vehicle wish to route
def add_ev(graph):
    # Generate vehicle
    traci.route.add('placeholder_trip', ['gneE53', 'gneE46'])
    traci.vehicle.add('EV1', 'placeholder_trip', typeID='electricvehicle')
    traci.vehicle.setParameter('EV1', 'device.battery.actualBatteryCapacity', '600')        # Set vehicles fuel at start

    # Generates optimal route for EV
    route = rerouter('gneE53', '-gneE64', 600, graph)
    traci.vehicle.setRoute('EV1', route)

# Adds vehicle type electric vehicle
def add_ev_vtype():
    original_stdout = sys.stdout

    # Get all lines in routing file apart from </route> so can append more to file
    f = open("data/electricvehicles.rou.xml", "r")
    lines = f.read()
    lines = lines.replace("</routes>", "")

    with open("data/electricvehicles.rou.xml", "w") as routes:
        sys.stdout = routes
        print(lines)
        print("""  <vType id="electricvehicle" accel="0.8" decel="4.5" sigma="0.5" minGap="2.5" maxSpeed="40" emissionClass="Energy/unknown" guiShape="evehicle">
                     <param key="has.battery.device" value="true"/>
                     <param key="maximumBatteryCapacity" value="2000"/>
                     <param key="maximumPower" value="1000"/>
                     <param key="vehicleMass" value="1000"/>
                     <param key="frontSurfaceArea" value="5"/>
                     <param key="airDragCoefficient" value="0.6"/>
                     <param key="internalMomentOfInertia" value="0.01"/>
                     <param key="radialDragCoefficient" value="0.5"/>
                     <param key="rollDragCoefficient" value="0.01"/>
                     <param key="constantPowerIntake" value="100"/>
                     <param key="propulsionEfficiency" value="0.9"/>
                     <param key="recuperationEfficiency" value="0.9"/>
                     <param key="stoppingTreshold" value="0.1"/>
             </vType>""")
         #print('     <vehicle id="EV1" type="electricvehicle" depart="0" route="circuit" ><param key="actualBatteryCapacity" value="5"/></vehicle>')
        print("</routes>")
        sys.stdout = original_stdout
