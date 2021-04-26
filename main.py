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

def run(netFile, additionalFile, options=None):
    """execute the TraCI control loop"""
    step = 0
    graph = Graph(netFile, additionalFile)
    mWhList = []
    mainStart, mainEnd = getEVMainStartEnd(netFile)
    evMainErrorCount = 0

    # EV outputs
    params = {}
    outputs = {}

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Add random EV routes
        if step >= 0 and step <= 140 \
           and step % 10 == 0:
            fromEdge = getEVEdges(graph, "")
            toEdge = getEVEdges(graph, fromEdge)

            add_ev(graph, fromEdge, toEdge, str(step), options, random.randint(200, 3000))

        if step == 150:
            params, algRuntime, csStops = add_ev(graph, mainStart, mainEnd, 'Main', options, 500)
            outputs["algRuntime"] = algRuntime
            outputs["params"] = params
            outputs["csStops"] = csStops

        # Gets output params for EV_Main
        if step >= 151:
            try:
                outputs["evBatteryCapacity"] = float(traci.vehicle.getParameter('EV_Main', 'device.battery.actualBatteryCapacity'))
                outputs["evDistance"] = float(traci.vehicle.getDistance('EV_Main'))
                outputs["evDuration"] = float(traci.vehicle.getLastActionTime('EV_Main'))
                outputs["evDuration"] -= 150
                outputs["lastEdge"] = traci.vehicle.getRoadID('EV_Main')

                if outputs["evBatteryCapacity"] <= 0:
                    print('EV_MAIN battery out of charge')
            except:
                evMainErrorCount += 1
                if evMainErrorCount >= 5:
                    break

        step += 1

    print('EV_Main Capacity at end: ', outputs["evBatteryCapacity"])
    print('EV_Main Distance: ', outputs["evDistance"])
    print('EV_Main Routing Duration: ', outputs["evDuration"])

    outputVehicleEndInfo(outputs)

    traci.close()
    sys.stdout.flush()

# Adds electric vehicle wish to route
def add_ev(graph, fromEdge, toEdge, evName, options, startingCapacity):
    vehicleID = 'EV_' + evName
    params = buildHyperParams(startingCapacity)
    algRuntime = ""
    csStops = []

    # Generate vehicle
    traci.route.add('placeholder_trip_' + evName, [fromEdge])
    traci.vehicle.add(vehicleID, 'placeholder_trip_' + evName, typeID='electricvehicle')
    traci.vehicle.setParameter(vehicleID, 'device.battery.actualBatteryCapacity', params["batteryCapacity"])

    # Run detour algorithm on EV or not
    if not options.noalg:
        start_time = time.time()
        route, csStops = rerouter(fromEdge, toEdge, vehicleID, graph, params)
        algRuntime = str(time.time() - start_time)
        print("Reroute algorithm runtime ", vehicleID, ": ", algRuntime)

        if len(route) > 0:
            traci.vehicle.setRoute(vehicleID, route)

            for chargingStation in csStops:
                traci.vehicle.setChargingStationStop(vehicleID, chargingStation.id, duration=chargingStation.Duration)

    else:
        start_time = time.time()
        print('fromEdge: ', fromEdge)
        print('toEdge: ', toEdge)
        route = traci.simulation.findRoute(fromEdge, toEdge)
        traci.vehicle.setRoute(vehicleID, route.edges)
        algRuntime = str(time.time() - start_time)

        print("No algorithm runtime ", vehicleID, ": ", algRuntime)

    return params, algRuntime, csStops

# Adds vehicle type electric vehicle
def add_ev_vtype():
    original_stdout = sys.stdout

    f = open("data/electricvehicles.rou.xml", "r+")
    f.truncate(0)       # Clear file
    f.close()

    with open("data/electricvehicles.rou.xml", "w") as routes:
        sys.stdout = routes
        print("<routes>")
        print("""  <vType id="electricvehicle" accel="0.8" decel="4.5" sigma="0.5" emissionClass="Energy/unknown" minGap="2.5" maxSpeed="40" guiShape="evehicle" vClass="evehicle">
                     <param key="has.battery.device" value="true"/>
                     <param key="maximumBatteryCapacity" value="10000"/>
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

def buildHyperParams(startingCapacity):
    hyperParams = {}

    hyperParams["DistanceFromStart"] = 0.15
    hyperParams["DistanceFromDivider"] = 0.15
    hyperParams["Price"] = 0.15
    hyperParams["VehiclesCharging"] = 0.40
    hyperParams["ChargePerStep"] = 0.15

    hyperParams["MinimumSoC"] = 10
    hyperParams["GoalCapacityAtEnd"] = 10
    hyperParams["batteryCapacity"] = startingCapacity

    return hyperParams

# Get run parameters
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="Run the commandline version of sumo")
    optParser.add_option("--noalg", action="store_true",
                         default=False, help="Do not run algorithm on simulation")
    optParser.add_option("--c", action="store", type="int",
                         default=1, help="Simulation run cycles")
    options, args = optParser.parse_args()
    return options

# Utility function to get random edges on network that allows EV vehicles
def getEVEdges(graph, otherEdge):
    while True:
        edge = random.choice(graph.Edges).getID()

        if graph.Net.getEdge(edge).allows('evehicle') and otherEdge != edge:
            break

    return edge

def outputVehicleEndInfo(outputs):
    duration = [cs.Duration for cs in outputs["csStops"]]

    csvRow = str(outputs["params"]).replace(",", "") + "," + str(outputs["evDistance"]) + ',' + \
             str(outputs["evDuration"]) + ',' + str(len(outputs["csStops"])) + ','+ \
             str(duration) + ','+ str(outputs["algRuntime"]) + ','+ \
             str(outputs["evBatteryCapacity"]) + ',' + str(outputs["lastEdge"])

    with open('data/EV_Outputs.csv', 'a+') as csv:
        # Get contents of the file to check if CSV headers should be added
        f = open("data/EV_Outputs.csv", "r+")
        contents = f.read()
        f.close()

        if len(contents) == 0:
            csv.write('Weightings,Duration,Route distance,CS stops,CS stop duration,Alg runtime,Capacity at end, Last edge\n')

        csv.write(csvRow + '\n')

def getEVMainStartEnd(netFile):
    if "EVGrid" in netFile:
        return "gneE53", "-gneE64"

    return "122066614#0", "167121171#7"
