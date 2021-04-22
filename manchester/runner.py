# from main import run, get_options
import os, sys, inspect
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

def generate_trips():
    randomTrips.main(randomTrips.get_options([
        '-n', 'data/EVGrid.net.xml',
        '--route-file', 'data/electricvehicles.rou.xml',
        '-t', 'data/electricvehicles.trip.xml',
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

    # Generates electric vehicle route and random trips
    main.add_ev_vtype()
    #generate_trips()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/osm.sumocfg",
                             "--tripinfo-output", "tripinfo.xml", "--additional-files", "data/Manchester_additionals.add.xml",
                             "--chargingstations-output", "data/Manchester_chargingstations.xml", "--no-warnings"])
    main.run(netFile='data/osm.net.xml',
             additionalFile='data/Manchester_additionals.add.xml',
             options=options)
