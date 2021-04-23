import os
import sys
import optparse
import random
from algorithm.reroute import rerouter, estimateRange
from algorithm.Graph import Graph
import time
import statistics

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib

# EVGrid route
# 'gneE53' -> '-gneE64'

# Manchester route
# '122066614#0' -> '167121171#7'

def run(netFile, additionalFile, options=None):
    """execute the TraCI control loop"""
    step = 0
    graph = Graph(netFile, additionalFile)
    mWhList = []
    eBatteryCapacity = 0.00
    evDistance = 0.00
    evTime = 0.00

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Add random EV routes
        if step >= 0 and step <= 100 \
           and step % 10 == 0:
            fromEdge = getEVEdges(graph, "")
            toEdge = getEVEdges(graph, fromEdge)

            add_ev(graph, fromEdge, toEdge, str(step))

        if step == 150:
            add_ev(graph, 'gneE53', '-gneE64', 'Main')

        if step >= 151:
            try:
                eBatteryCapacity = float(traci.vehicle.getParameter('EV_Main', 'device.battery.actualBatteryCapacity'))
                evDistance = float(traci.vehicle.getDistance('EV_Main'))
                evTime = float(traci.vehicle.getLastActionTime('EV_Main'))
            except:
                print("EV_Main not found")

        step += 1

    print('EV Capacity at end: ', eBatteryCapacity)
    print('EV Distance: ', evDistance)
    print('EV Routing Duration: ', evTime - 150)

    traci.close()
    sys.stdout.flush()

# Adds electric vehicle wish to route
def add_ev(graph, fromEdge, toEdge, evName):
    vehicleID = 'EV_' + evName
    hyperParams = buildHyperParams()
    batteryCapacity = hyperParams["batteryCapacity"]

    # Generate vehicle
    traci.route.add('placeholder_trip_' + evName, [toEdge])
    traci.vehicle.add(vehicleID, 'placeholder_trip_' + evName, typeID='electricvehicle')
    traci.vehicle.setParameter(vehicleID, 'device.battery.actualBatteryCapacity', batteryCapacity)

    # Generates optimal route for EV
    start_time = time.time()
    route, csStops = rerouter(fromEdge, toEdge, vehicleID, graph, hyperParams)
    print("Reroute algorithm runtime ", vehicleID, ": ", str(time.time() - start_time))

    if len(route) > 0:
        traci.vehicle.setRoute(vehicleID, route)

        for chargingStation in csStops:
            traci.vehicle.setChargingStationStop(vehicleID, chargingStation.id, duration=chargingStation.Duration)

# Adds vehicle type electric vehicle
def add_ev_vtype():
    original_stdout = sys.stdout

    f = open("data/electricvehicles.rou.xml", "r+")
    f.truncate(0)       # Clear file

    with open("data/electricvehicles.rou.xml", "w") as routes:
        sys.stdout = routes
        print("<routes>")
        print("""  <vType id="electricvehicle" accel="0.8" decel="4.5" sigma="0.5" emissionClass="Energy/unknown" minGap="2.5" maxSpeed="40" guiShape="evehicle" vClass="evehicle">
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
                     <param key="has.tripinfo.device" value="true"/>
                   </vType>""")
        print("</routes>")
        sys.stdout = original_stdout

def buildHyperParams():
    hyperParams = {}

    hyperParams["DistanceFromStart"] = 0.32
    hyperParams["DistanceFromDivider"] = 0.32
    hyperParams["Price"] = 0.12
    hyperParams["VehiclesCharging"] = 0.12
    hyperParams["ChargePerStep"] = 0.12

    hyperParams["MinimumSoC"] = 10
    hyperParams["batteryCapacity"] = 300

    return hyperParams

# Get run parameters
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="Run the commandline version of sumo")
    optParser.add_option("--algorithm", action="store_true",
                         default=False, help="Run algorithm on simulation")
    options, args = optParser.parse_args()
    return options

# Utility function to get random edges on network that allows EV vehicles
def getEVEdges(graph, otherEdge):
    while True:
        edge = random.choice(graph.Edges).getID()

        if graph.Net.getEdge(edge).allows('evehicle') and otherEdge != edge:
            break

    return edge

def outputVehicleEndInfo():
    return ""
