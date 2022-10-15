# EVDetourRouting
Efficient Detour Computation Scheme for Electric Vehicles

[mdpi.com/2079-9292/11/5/803](mdpi.com/2079-9292/11/5/803)

## Usage (Tested on Windows 10 Pro V: 20H2)

#### Installation

* https://www.python.org/downloads/
* https://sumo.dlr.de/docs/Downloads.php (Minimum version: 1.9.0)

Contains two scenarios to simulate algorith:

- EVGrid
- manchester

Both scenarios have an initial 'runner.py' file that runs the simulation. Make sure you have an environment set-up for SUMO_HOME.

Typical workflow for simulation running:

```
cd EVGrid
// Without SUMO gui
python runner.py --nogui

// With sumo gui
python runner.py --nogui

// Without algorithm
python runner.py --nogui --noalg
```

Amount of times simulation is run can be changed through the '--c' console option, default value is 1. Also EVs injected into simulation can be toggled through '--v', default value is 50.

Output for all vehicles in the simulation is in the 'data/TripInfo.xml' and output for the EVs inputted into the simulation in the 'data/EV_Outputs.csv'.
