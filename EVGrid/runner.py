# from main import run, get_options
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
import main
import optparse
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import randomTrips  # noqa

def generate_ev():
    random.seed(42)  # make tests reproducible
    # N = 3600  # nmber of time steps
    with open("data/electricvehicles.rou.xml", "w") as routes:
        print("""<routes>
            <vType id="electricvehicle" accel="0.8" decel="4.5" sigma="0.5" minGap="2.5" maxSpeed="40" guiShape="passenger" emissionClass="Energy/unknown">
                    <param key="has.battery.device" value="true"/>
                    <param key="maximumBatteryCapacity" value="25"/>
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
            </vType>

            <route id="circuit" edges="gneE0 gneE4 gneE5 gneE6 gneE7 gneE8 gneE9 gneE10"/>
            """, file=routes)
        vehNr = 0
        print('     <vehicle id="EV1" type="electricvehicle" depart="0" route="circuit" ><param key="actualBatteryCapacity" value="5"/></vehicle>', file=routes)
        print("</routes>", file=routes)

def generate_trips():
    randomTrips.main(randomTrips.get_options([
        '-n', 'data/EVGrid.net.xml',
        '--route-file', 'data/electricvehicles.rou.xml',
        #'-t', 'data/electricvehicles.trip.xml',
        '--prefix', 'EV',
        '-e', '10',
        '-p', '1.93',
        '--flows', '100',
        '--random'
    ]))

# Script entry point
if __name__ == "__main__":
    options = main.get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # Generates electric vehicles in route file
    generate_ev()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/EVGrid.sumocfg",
                             "--tripinfo-output", "tripinfo.xml", "--emission-output", "emission-output.xml"])
    main.run(options)
