import os
import sys
import optparse
import random

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

    # traci.route.add('placeholder_trip', ["gneE53", "gneE46"])
    # traci.vehicle.add('EV1', 'placeholder_trip', typeID='electricvehicle')
    # traci.vehicle.setParameter('EV1', 'device.battery.actualBatteryCapacity', '200')
    #
    # route = get_optimal_route_ev('EV1')
    # traci.route.add('trip', route)
    # traci.vehicle.setRoute('EV1', 'trip')

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # Checks EV battery capacity
        # print(traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity'))
        # if traci.vehicle.getParameter('EV1', 'device.battery.actualBatteryCapacity') == "0.00":
        #     print('Vehicle battery empty')
        #     #break

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

# Generates optimal route for electric vehicle
def get_optimal_route_ev(vehicleID):
    #Hardcoded route for now
    # traci.vehicle.setStop(vehicleID, "chargingStation_gneE46_0_0")
    return ["gneE53", "gneE46"]

# Adds vehicle type electric vehicle
def add_ev_vtype():
    random.seed(42)  # make tests reproducible
    # N = 3600  # nmber of time steps

    original_stdout = sys.stdout

    with open("data/electricvehicles.rou.xml", "w") as routes:
         sys.stdout = routes
         print("""<routes>
            <vType id="electricvehicle" accel="0.8" decel="4.5" sigma="0.5" minGap="2.5" maxSpeed="40" emissionClass="Energy/unknown" guiShape="evehicle">
                     <param key="has.battery.device" value="true"/>
                     <param key="maximumBatteryCapacity" value="200"/>
                     <param key="maximumPower" value="1000"/>
                     <param key="vehicleMass" value="10000"/>
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
