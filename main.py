import os
import sys
import optparse
import random
from algorithm.reroute import rerouter, estimateRange
from algorithm.Graph import Graph
import time
import statistics
from bs4 import BeautifulSoup

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib

globalSeed = 0

def run(netFile, additionalFile, options=None, batteryCapacity=None, paramType=None, seed=None):
    """execute the TraCI control loop"""
    step = 0
    graph = Graph(netFile, additionalFile)
    mWhList = []
    evMainErrorCount = 0

    global globalSeed
    globalSeed = seed * 10000

    # EV outputs
    params = {}
    outputs = {}
    evs = []

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        upperVehicleLimit = (options.v * 10) + 199

        # Add random EV routes
        if step >= 200 and step <= upperVehicleLimit and step % 10 == 0:
            fromEdge = getEVEdges(graph, "")
            toEdge = getEVEdges(graph, fromEdge)
            evName = 'EV_' + str(step)

            algRuntime, csStops = add_ev(graph, fromEdge, toEdge, evName, options, batteryCapacity, paramType)
            evs.append(evName)
            outputs[evName] = {}

            outputs[evName]["algRuntime"] = algRuntime
            outputs[evName]["paramType"] = paramType
            outputs[evName]["startingBatteryCapacity"] = batteryCapacity
            outputs[evName]["csStops"] = csStops
            outputs[evName]["Start"] = fromEdge
            outputs[evName]["End"] = toEdge

        step += 1

    outputVehicleEndInfo(outputs, evs)

    traci.close()
    sys.stdout.flush()

# Adds electric vehicle wish to route
def add_ev(graph, fromEdge, toEdge, evName, options, startingCapacity, paramType):
    vehicleID = evName
    params = buildHyperParams(startingCapacity, paramType)
    algRuntime = ""
    csStops = []

    print('params: ', params)

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

        route = traci.simulation.findRoute(fromEdge, toEdge)
        traci.vehicle.setRoute(vehicleID, route.edges)
        algRuntime = str(time.time() - start_time)

        print("No algorithm runtime ", vehicleID, ": ", algRuntime)

    return algRuntime, csStops

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

# Get hyper parameters used in the simualation
def buildHyperParams(startingCapacity, paramType):
    hyperParams = {}

    hyperParams["A"] = {"DistanceFromStart": 0.35, "DistanceFromDivider": 0.35, "Price": 0.10, "VehiclesCharging": 0.10, "ChargePerStep": 0.10, "MinimumSoC": 10, "GoalCapacityAtEnd": 10, "batteryCapacity": startingCapacity}
    hyperParams["B"] = {"DistanceFromStart": 0.10, "DistanceFromDivider": 0.10, "Price": 0.6, "VehiclesCharging": 0.10, "ChargePerStep": 0.10, "MinimumSoC": 10, "GoalCapacityAtEnd": 10, "batteryCapacity": startingCapacity}
    hyperParams["C"] = {"DistanceFromStart": 0.10, "DistanceFromDivider": 0.10, "Price": 0.10, "VehiclesCharging": 0.6, "ChargePerStep": 0.10, "MinimumSoC": 10, "GoalCapacityAtEnd": 10, "batteryCapacity": startingCapacity}
    hyperParams["D"] = {"DistanceFromStart": 0.10, "DistanceFromDivider": 0.10, "Price": 0.10, "VehiclesCharging": 0.10, "ChargePerStep": 0.6, "MinimumSoC": 10, "GoalCapacityAtEnd": 10, "batteryCapacity": startingCapacity}
    hyperParams["E"] = {"DistanceFromStart": 0.2, "DistanceFromDivider": 0.2, "Price": 0.2, "VehiclesCharging": 0.2, "ChargePerStep": 0.2, "MinimumSoC": 10, "GoalCapacityAtEnd": 10, "batteryCapacity": startingCapacity}

    return hyperParams[paramType]

# Get run parameters
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="Run the commandline version of sumo")
    optParser.add_option("--noalg", action="store_true",
                         default=False, help="Do not run algorithm on simulation")
    optParser.add_option("--c", action="store", type="int",
                         default=1, help="Simulation run cycles")
    optParser.add_option("--v", action="store", type="int",
                         default=50, help="EVs injected into simualation")
    options, args = optParser.parse_args()
    return options

# Utility function to get random edges on network that allows EV vehicles
def getEVEdges(graph, otherEdge):
    while True:
        global globalSeed
        globalSeed += 1
        random.seed(globalSeed)

        edge = random.choice(graph.Edges).getID()
        startNode = graph.Net.getEdge(edge).getToNode().getID()

        try:
            endNode = graph.Net.getEdge(otherEdge).getFromNode().getID()
        except:
            endNode = ""

        if graph.Net.getEdge(edge).allows('evehicle') and otherEdge != edge \
            and startNode != endNode:
            break
        else:
            globalSeed += 1

    return edge

# Outputs data for all EVs in simulation
def outputVehicleEndInfo(outputs, evs):
    trip_content = getXmlContent("data/tripinfo.xml")
    battery_content = getXmlContent("data/battery.out.xml")

    with open('data/EV_Outputs.csv', 'a+') as csv:
        # Get contents of the file to check if CSV headers should be added
        f = open("data/EV_Outputs.csv", "r+")
        contents = f.read()
        f.close()

        if len(contents) == 0:
            csv.write('Weightings,Starting Battery Capacity (Wh),Route Distance (m),Travel Time (s),CS stops,CS Stop Duration (s),Algorithm Runtime (s),Remaining Battery Capacity At End (Wh),Number of EVs Charging,Start Edge,End Edge\n')

        for e in evs:
            duration = [cs.Duration for cs in outputs[e]["csStops"]]
            evCharging = [cs.VehiclesCharging for cs in outputs[e]["csStops"]]

            trip_EV = trip_content.find(id=e)
            battery_EV = battery_content.findAll(id=e)

            try:
                batteryCap = battery_EV[-1]["actualbatterycapacity"]
            except:
                try:
                    batteryCap = battery_EV[-2]["actualbatterycapacity"]
                except:
                    batteryCap = 0

            csvRow = str(outputs[e]["paramType"]) + "," + str(outputs[e]["startingBatteryCapacity"]) + "," + str(trip_EV["routelength"]) + ',' + \
                     str(trip_EV["duration"]) + ',' + str(len(outputs[e]["csStops"])) + ','+ \
                     str(duration) + ','+ str(outputs[e]["algRuntime"]) + ','+ \
                     str(batteryCap) + ',' + str(evCharging) + ',' + str(outputs[e]["Start"]) + ',' + str(outputs[e]["End"])

            csv.write(csvRow + '\n')

# Clear the EV output file
def clearOutput():
    f = open("data/EV_Outputs.csv", "r+")
    f.truncate(0)
    f.close()

# Get contents of an XML file
def getXmlContent(file):
    content = []
    # Read the XML file
    with open(file, "r") as file:
        # Read each line in the file, readlines() returns a list of lines
        content = file.readlines()
        # Combine the lines in the list into a string
        content = "".join(content)
        out_content = BeautifulSoup(content, "lxml")

    return out_content
