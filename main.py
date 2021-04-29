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

def run(netFile, additionalFile, options=None, batteryCapacity=None):
    """execute the TraCI control loop"""
    step = 0
    graph = Graph(netFile, additionalFile)
    mWhList = []
    mainStart, mainEnd = getEVMainStartEnd(netFile)
    evMainErrorCount = 0

    # EV outputs
    params = {}
    outputs = {}
    evs = []

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Add random EV routes
        if step >= 200 and step <= 700 and step % 10 == 0:
            fromEdge = getEVEdges(graph, "")
            toEdge = getEVEdges(graph, fromEdge)
            evName = 'EV_' + str(step)

            params, algRuntime, csStops = add_ev(graph, fromEdge, toEdge, evName, options, batteryCapacity)
            evs.append(evName)
            outputs[evName] = {}

            outputs[evName]["algRuntime"] = algRuntime
            outputs[evName]["params"] = params
            outputs[evName]["csStops"] = csStops

        step += 1

    outputVehicleEndInfo(outputs, evs)

    traci.close()
    sys.stdout.flush()

# Adds electric vehicle wish to route
def add_ev(graph, fromEdge, toEdge, evName, options, startingCapacity):
    vehicleID = evName
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

    hyperParams["DistanceFromStart"] = 0.2
    hyperParams["DistanceFromDivider"] = 0.2
    hyperParams["Price"] = 0.2
    hyperParams["VehiclesCharging"] = 0.2
    hyperParams["ChargePerStep"] = 0.20

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

def outputVehicleEndInfo(outputs, evs):
    trip_content = getXmlContent("data/tripinfo.xml")
    battery_content = getXmlContent("data/battery.out.xml")

    with open('data/EV_Outputs.csv', 'a+') as csv:
        # Get contents of the file to check if CSV headers should be added
        f = open("data/EV_Outputs.csv", "r+")
        contents = f.read()
        f.close()

        if len(contents) == 0:
            csv.write('Weightings,Route distance,Duration,CS stops,CS stop duration,Alg runtime,Capacity at end,EVs Charging\n')

        for e in evs:
            duration = [cs.Duration for cs in outputs[e]["csStops"]]
            evCharging = [cs.VehiclesCharging for cs in outputs[e]["csStops"]]

            trip_EV = trip_content.find(id=e)
            battery_EV = battery_content.findAll(id=e)

            try:
                batteryCap = battery_EV[-1]["actualbatterycapacity"]
            except:
                batteryCap = battery_EV[-2]["actualbatterycapacity"]

            csvRow = str(outputs[e]["params"]).replace(",", "") + "," + trip_EV["routelength"] + ',' + \
                     trip_EV["duration"] + ',' + str(len(outputs[e]["csStops"])) + ','+ \
                     str(duration) + ','+ str(outputs[e]["algRuntime"]) + ','+ \
                     batteryCap + ',' + str(evCharging)

            csv.write(csvRow + '\n')

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

def getEVMainStartEnd(netFile):
    if "EVGrid" in netFile:
        return "gneE53", "-gneE64"

    return "122066614#0", "167121171#7"
