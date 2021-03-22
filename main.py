import os
import sys
import optparse
import random
from algorithm.reroute import rerouter, estimateRange

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib

def run(netFile, additionalFile, options=None):
    """execute the TraCI control loop"""
    step = 0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if step == 100:
            add_ev(netFile, additionalFile)

        # Checks EV battery capacity
        # if step > 105:
        #     mWh = float(traci.vehicle.getDistance('EV1')) / float(traci.vehicle.getParameter('EV1', 'device.battery.totalEnergyConsumed'))
        #     print('MwH: ', mWh)
            # print('Range: ', float(traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity')) * mWh)
            # print('actualBatteryCapacity: ', float(traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity')))
        # if traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity') == "0.00":
        #     print('Vehicle battery empty')

        step += 1

    traci.close()
    sys.stdout.flush()

# Adds electric vehicle wish to route
def add_ev(netFile, additionalFile):
    vehicleID = 'EV1'
    batteryCapacity = 90

    # Generate vehicle
    traci.route.add('placeholder_trip', ['gneE53'])
    # traci.route.add('placeholder_trip', ['27252673#2', '27252673#2'])
    traci.vehicle.add(vehicleID, 'placeholder_trip', typeID='electricvehicle')
    traci.vehicle.setParameter('EV1', 'device.battery.actualBatteryCapacity', batteryCapacity)

    # Generates optimal route for EV
    route, csStops = rerouter('gneE53', '-gneE64', vehicleID, netFile, additionalFile)
    # route, csStops = reroute.rerouter('27252673#2', '167121167#2', batteryCapacity, graph)

    if len(route) > 0:
        traci.vehicle.setRoute(vehicleID, route)

        for chargingStation in csStops:
            traci.vehicle.setChargingStationStop(vehicleID, chargingStation.id, duration=chargingStation.Duration)

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
        print("""  <vType id="electricvehicle" accel="0.8" decel="4.5" sigma="0.5" emissionClass="Energy/unknown" minGap="2.5" maxSpeed="40" guiShape="evehicle">
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

# Get run parameters
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="Run the commandline version of sumo")
    optParser.add_option("--algorithm", action="store_true",
                         default=False, help="Run algorithm on simulation")
    options, args = optParser.parse_args()
    return options
