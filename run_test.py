#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from cbswh import CBSWHSolver
from cbswdgh import CBSWDGHSolver
from cbswhl import CBSWHLSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import time

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(agents, instance, num):
    f = Path(instance)
    a = Path(agents)
    if not f.is_file():
        raise BaseException(instance + " does not exist.")
    if not a.is_file():
        raise BaseException(agents + " does not exist.")
    f = open(instance, 'r')
    a = open(agents, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for i in range(len(line)-1):
            if line[i] == '@':
                my_map[-1].append(True)
            elif line[i] == '.':
                my_map[-1].append(False)
            else:
                my_map[-1].append(False)
    # #agents
    line = a.readline()
    num_agents = int(num)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    s = 0
    while s < num_agents:
        line = a.readline()
        arrLine = [x for x in line.split('\t')]
        sx = int(arrLine[4])
        sy = int(arrLine[5])
        gx = int(arrLine[6])
        gy = int(arrLine[7])
        if not (my_map[sx][sy] or my_map[gx][gy]):
            starts.append((int(sx), int(sy)))
            goals.append((int(gx), int(gy)))
            s += 1
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--agents', type=str, default=None,
                        help='The name of the agent file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--num', type=int, default=2,
                        help='The number of agents, defaults to 2')

    args = parser.parse_args()


    result_file = open("results" + args.solver + ".csv", "w", buffering=1)
    result_file.write("{},{},{},{},{}\n".format('agents', 'cost', 'time', 'generate', 'expande'))

    for agents in sorted(glob.glob(args.agents)):

        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(agents, args.instance, args.num)
        print_mapf_instance(my_map, starts, goals)

        # start time
        starttime = time.time()
        num_of_generated = "Unknown"
        num_of_expanded = "Unknown"

        if args.solver == "CBS":
            print("***Run CBS***")
            cbs = CBSSolver(my_map, starts, goals)
            [paths, num_of_generated, num_of_expanded] = cbs.find_solution(args.disjoint)
        elif args.solver == "CBSWH":
            print("***Run CBSWH***")
            cbswh = CBSWHSolver(my_map, starts, goals)
            [paths, num_of_generated, num_of_expanded] = cbswh.find_solution(args.disjoint)
        elif args.solver == "CBSWDGH":
            print("***Run CBSWDGH***")
            cbswdgh = CBSWDGHSolver(my_map, starts, goals)
            [paths, num_of_generated, num_of_expanded] = cbswdgh.find_solution(args.disjoint)
        elif args.solver == "CBSWHL":
            print("***Run CBSWHL***")
            cbswhl = CBSWHLSolver(my_map, starts, goals)
            [paths, num_of_generated, num_of_expanded] = cbswhl.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "CBSdisjoint":
            print("***Run CBSdisjoint***")
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution_disjoint(args.disjoint)
        else:
            raise RuntimeError("Unknown solver!")

        # end time
        endtime = time.time()
        # run time
        runtime = round(endtime - starttime, 2)

        cost = get_sum_of_cost(paths)
        result_file.write("{},{},{},{},{}\n".format(agents, cost, runtime, num_of_generated, num_of_expanded))


        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
