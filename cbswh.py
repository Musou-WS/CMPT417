import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import copy
import itertools


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    time = max(len(path1), len(path2))
    for i in range(time):
        loc1 = [get_location(path1, i-1), get_location(path1, i)]
        loc2 = [get_location(path2, i-1), get_location(path2, i)]
        # vertex collision
        if loc1[1] == loc2[1]:
            return {'loc': [loc1[1]], 'timestep': i}
        elif (loc1[0] == loc2[1]) & (loc1[1] == loc2[0]):
            return {'loc': [loc1[0], loc1[1]], 'timestep': i}
        else:
            continue
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)):
        for j in range(len(paths)-1, i, -1):
            collision = detect_collision(paths[i], paths[j])
            if not (collision == None):
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
    constraints.append({'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': False})
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    constraints = []
    randomNum = random.randint(0,1)
    if randomNum:
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True})
    else:
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': True})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep'], 'positive': False})
    return constraints

#find 2-approximation of min_vertex_cover
def min_vertex_cover(edges):
    c=[]
    count = 0
    while len(edges) > 0:
        next_edge = edges.pop()
        count += 1
        for edge in edges:
            if (edge[0] == next_edge[0]) or (edge[0] == next_edge[1]) or (edge[1] == next_edge[0]) or (edge[1] == next_edge[1]):
                edges.remove(edge)
    return count


def H_DG(collisions):
    count = 0
    while len(collisions) > 0:
        next_collision = collisions.pop()
        count += 1
        for collision in collisions:
            if (collision['a1'] == next_collision['a1']) or (collision['a1'] == next_collision['a2']) or (collision['a2'] == next_collision['a1']) or (collision['a2'] == next_collision['a2']):
                collisions.remove(collision)
    return count

# Brute-Force
def H_DG_better(collisions, agentsNum):
    count = 0
    edges = []
    vers = [n for n in range(agentsNum)]
    for collision in collisions:
        edges.append([collision['a1'], collision['a2']])
    #forward
    for num in range(len(vers)):
        for versSelect in itertools.combinations(vers, num):
            edgesTemp = copy.deepcopy(edges)
            for edge in edges:
                if (edge[0] in versSelect) or (edge[1] in versSelect):
                    edgesTemp.remove(edge)
            if len(edgesTemp) == 0:
                count = num
                return num
    return count


class CBSWHSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost']+node['h'], node['h'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node
        
    # disjoint
    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'h': 0}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            next_node = self.pop_node()
            if len(next_node['collisions']) == 0:
                return [next_node['paths'], self.num_of_generated, self.num_of_expanded]
            else:
                next_constraints = []
                if disjoint:
                    next_constraints = disjoint_splitting(next_node['collisions'][0])
                else:
                    next_constraints = standard_splitting(next_node['collisions'][0])
                for next_constraint in next_constraints:
                    new_node = {'cost': 0,
                        'constraints': [],
                        'paths': [],
                        'collisions': [],
                        'h': 0}
                    new_node['constraints'] = copy.deepcopy(next_node['constraints'])
                    new_node['constraints'].append(next_constraint)
                    new_node['paths'] = copy.deepcopy(next_node['paths'])
                    new_agent = next_constraint['agent']
                    if next_constraint['positive']:
                        for i in range(self.num_of_agents):
                            new_path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                              i, new_node['constraints'])
                            if new_path is not None:
                                new_node['paths'][i] = copy.deepcopy(new_path)
                                if i == (self.num_of_agents-1):
                                    new_node['collisions'] = detect_collisions(new_node['paths'])
                                    new_node['cost'] = get_sum_of_cost(new_node['paths'])
                                    new_node['h'] = H_DG_better(copy.deepcopy(new_node['collisions']), len(new_node['paths']))
                                    self.push_node(new_node)
                            else:
                                break
                    else:
                        new_path = a_star(self.my_map, self.starts[new_agent], self.goals[new_agent], self.heuristics[new_agent],
                                          new_agent, new_node['constraints'])
                        if new_path is not None:
                            new_node['paths'][new_agent] = copy.deepcopy(new_path)
                            new_node['collisions'] = detect_collisions(new_node['paths'])
                            new_node['cost'] = get_sum_of_cost(new_node['paths'])
                            new_node['h'] = H_DG_better(copy.deepcopy(new_node['collisions']), len(new_node['paths']))
                            self.push_node(new_node)
        raise BaseException('No solutions')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
