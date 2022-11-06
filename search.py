# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import math

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


class node:
    def __init__(self, position, direction, cost, parent):
        self.position = position
        self.direction = direction
        self.cost = cost
        self.parent = parent

    def __str__(self):
        return "Node{" + "position = " + str(self.position) + ", direction = " + str(
            self.direction) + ", cost = " + str(self.cost) + ", parent = " + str(self.parent.position)


def universalSearch(problem, dataStructure, priorityFunction=None, heuristic=None):
    explored = set()
    currentNode = node(problem.getStartState(), 'Stop', 0, None)
    explored.add(currentNode)
    for position, direction, cost in problem.getSuccessors(currentNode.position):
        if priorityFunction is not None:
            newCost = priorityFunction(problem, currentNode, cost, heuristic)
            dataStructure.push(node(position, direction, newCost, currentNode), newCost)
        else:
            newCost = currentNode.cost + cost
            dataStructure.push(node(position, direction, newCost, currentNode))
    while True:
        if dataStructure.isEmpty():
            return False
        currentNode = dataStructure.pop()
        explored.add(currentNode)
        if problem.isGoalState(currentNode.position):
            break
        for position, direction, cost in problem.getSuccessors(currentNode.position):
            if position not in {n.position for n in explored}:
                if priorityFunction is not None:
                    newCost = priorityFunction(problem, currentNode, cost, heuristic)
                    dataStructure.push(node(position, direction, newCost, currentNode), newCost)
                else:
                    newCost = currentNode.cost + cost
                    dataStructure.push(node(position, direction, newCost, currentNode))
    result = []
    while currentNode.parent is not None:
        result.insert(0, currentNode.direction)
        currentNode = currentNode.parent
    return result


def depthFirstSearch(problem):
    return universalSearch(problem, util.Stack())


def breadthFirstSearch(problem):
    return universalSearch(problem, util.Queue())


def priorityUCS(problem, currentNode, cost, heuristic):
    return currentNode.cost + cost


def uniformCostSearch(problem):
    return universalSearch(problem, util.PriorityQueue(), priorityFunction=priorityUCS)


def nullHeuristic(state, problem):
    dx = state[0] - problem.goal[0]
    dy = state[1] - problem.goal[1]
    return math.sqrt(dx * dx + dy * dy)


def incrementWithGoAround(x, y, incrementx, incrementy, walls):
    nextx = x + incrementx
    nexty = y + incrementy
    if nextx < 0:
        nextx = walls.width - 1
    elif nextx == walls.width:
        nextx = 0

    if nexty < 0:
        nexty = walls.height - 1
    elif nexty == walls.height:
        nexty = 0
    return nextx, nexty


def nineWayHeuristic(state, problem):
    goals = []
    goals_dx = [problem.walls.width, -problem.walls.width, 0, 0,
                problem.walls.width, -problem.walls.width, problem.walls.width, -problem.walls.width]
    goals_dy = [0, 0, problem.walls.height, -problem.walls.height,
                problem.walls.height, -problem.walls.height, -problem.walls.height, problem.walls.height]
    goals.append(problem.goal)
    for dx, dy in zip(goals_dx, goals_dy):
        goals.append(incrementWithGoAround(problem.goal[0], problem.goal[1], dx, dy, problem.walls))

    distances = []
    for goal in goals:
        dx = state[0] - goal[0]
        dy = state[1] - goal[1]
        distances.append(math.sqrt(dx * dx + dy * dy))
        # distances.append(abs(goal[0] - state[0]) + abs(goal[1] - state[1]))
    distances.sort()
    return distances[0]


def priorityAStar(problem, currentNode, cost, heuristic):
    return priorityUCS(problem, currentNode, cost, heuristic) + heuristic(currentNode.position, problem)


def priorityAStarWeighted(problem, currentNode, cost, heuristic):
    return priorityUCS(problem, currentNode, cost, heuristic) + heuristic(currentNode.position, problem)


def aStarSearch(problem, heuristic=nullHeuristic):
    return universalSearch(problem, util.PriorityQueue(), priorityFunction=priorityAStar, heuristic=heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
