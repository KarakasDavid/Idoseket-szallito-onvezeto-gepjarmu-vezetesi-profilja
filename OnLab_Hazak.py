from __future__ import absolute_import
from __future__ import print_function

import optparse
import os

import csvfile as csvfile
import pandas as pd
import csv
import sys

#################################################################################################### - Sumo beállítások

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
from sklearn.model_selection import ParameterGrid
import traci  # noqa

#################################################################################################### - Sumo beállítások


# Tesztadatok listája, ezekből minden egyes teszt esetén beolvas egy sort és felhasználja őket
# majd a legjobb sort kiválasztja a run_grid_search metódussal
param_grid = [{'accel': [10.0], 'decel': [2.0], 'maxSpeed': [10.0]},
              {'accel': [7.5], 'decel': [3.0], 'maxSpeed': [12.0]},
              {'accel': [7.5], 'decel': [4.0], 'maxSpeed': [12.0]},
              {'accel': [6.5], 'decel': [5.0], 'maxSpeed': [14.0]},
              {'accel': [4.0], 'decel': [6.0], 'maxSpeed': [13.0]},
              {'accel': [1.0], 'decel': [2.0], 'maxSpeed': [7.0]}]



# könnyen iterálható forma
grid = ParameterGrid(param_grid)



# Lefutttat egy szimulációt az adott paraméterekkel
# jelen esetben ezeket az elején beállítja
# A futás ideje a for ciklusban állítható
# Jelenleg az átlagos várakozási időt nézi mint adatot amihez a legjobbat keresi
# Az a paraméter csoport lesz kiválasztva amelyinek a legkisebb az átlagos várakozási ideje
def evaluate_simulation(accel, decel, maxSpeed, br, br_1):
    traci.vehicle.setAccel("11", accel)
    traci.vehicle.setDecel("11", decel)
    traci.vehicle.setMaxSpeed("11", maxSpeed)

    traci.vehicle.setAccel("1", accel)
    traci.vehicle.setDecel("1", decel)
    traci.vehicle.setMaxSpeed("1", maxSpeed)

    break_count =0
    for i in range(300):
        for vehicle_id in traci.vehicle.getIDList():
            if vehicle_id == "11" and traci.vehicle.getAcceleration(vehicle_id) < -2.6:
                break_count += 1
            if vehicle_id == "1" and traci.vehicle.getDecel(vehicle_id) > 0.0:
                br_1 += 1
        traci.simulationStep()

    total_waiting_time = 0
    num_vehicles = traci.vehicle.getIDCount()
    for i in range(num_vehicles):
        vehicle_id = traci.vehicle.getIDList()[i]
        total_waiting_time += traci.vehicle.getAccumulatedWaitingTime(vehicle_id)

    avg_waiting_time = total_waiting_time / num_vehicles

    with open('adatok.csv', mode='a', newline='') as csvfile:

        # Create a csv writer object
        writer = csv.writer(csvfile, dialect='excel')

        # Write data to the file
        writer.writerow([accel, decel, maxSpeed, avg_waiting_time, ID+1])

    print('Average time:', avg_waiting_time)
    #print('Brake count:', break_count)
    #print('Brake count_1:', br_1)

    return avg_waiting_time


def run_grid_search(br, br_1):
    best_score = float('inf')
    best_params = grid[0]

    for params in grid:
        accel, decel, maxSpeed = params['accel'], params['decel'], params['maxSpeed']
        score = evaluate_simulation(accel, decel, maxSpeed, br, br_1)

        if score < best_score:
            best_score = score
            best_params = params
    print('Best time:',best_score)
    print('Best parameters:', best_params)

    # Calculate and return evaluation metric based on simulation results
    return best_params


def run(br, br_1):
    """execute the TraCI control loop"""
    step = 0
    szam = 0

    best_params = run_grid_search(br, br_1)

    accel, decel, maxSpeed = best_params['accel'], best_params['decel'], best_params['maxSpeed']
    traci.vehicle.setAccel("11", accel)
    traci.vehicle.setDecel("11", decel)
    traci.vehicle.setMaxSpeed("11", maxSpeed)

    while traci.simulation.getMinExpectedNumber() > 0 and szam < 1200:
        traci.simulationStep()
        # print(f"{szam}\n")
        szam += 1

    step += 1
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


#Main függvény
if __name__ == "__main__":
    options = get_options()

    # Sumohoz mint szerverhez kapcsolódik
    # majd futtatja
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    if os.path.exists('adatok.csv'):
        with open('adatok.csv', mode='r') as csv_file:
            # Create a CSV reader object
            csv_reader = csv.reader(csv_file)

            # Read one line from the CSV file
            row = next(csv_reader)

            # Display the row
            print(row[-1])
            ID = int(row[-1])+1
    else:
        print("Első bejegyzés")
        ID = -1
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start( [sumoBinary, "-c", "C:\SumoProjects - OnLab_Hazak/OnLab_Hazak.sumocfg" , "--lateral-resolution", "0.8","--random", "--random-depart-offset", "10", ])
    br = 0
    br_1 = 0
    run(br, br_1)


    print(f"Lefutott és itt a vége: !")



