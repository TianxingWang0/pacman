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

from util import Stack, Queue, PriorityQueue

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # fringe = Stack()
    # start_state = problem.getStartState()
    # fringe.push((start_state, []))  # (state, the path from start to current state)
    # visited = set()
    # while not fringe.isEmpty():
    #     curr, path = fringe.pop()
    #     if problem.isGoalState(curr):
    #         return path
    #     if curr in visited:
    #         continue
    #     visited.add(curr)
    #     for nbr, direction, cost in problem.getSuccessors(curr):
    #         if nbr not in visited:
    #             tmp = path + [direction]
    #             fringe.push((nbr, tmp))
    # return []

    fringe = Stack()
    start_state = problem.getStartState()
    fringe.push(start_state)
    fringe_path = dict()  # <state, optimal path from start to state>
    fringe_path[start_state] = []
    visited = set()
    while not fringe.isEmpty():
        curr = fringe.pop()
        path = fringe_path.pop(curr)
        if problem.isGoalState(curr):
            return path
        if curr in visited:
            continue
        visited.add(curr)
        for nbr, direction, cost in problem.getSuccessors(curr):
            if nbr not in visited:
                fringe.push(nbr)
                fringe_path[nbr] = path + [direction]
    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # fringe = Queue()
    # start_state = problem.getStartState()
    # fringe.push((start_state, []))  # (state, the path from start to current state)
    # visited = set()
    # while not fringe.isEmpty():
    #     curr, path = fringe.pop()
    #     if problem.isGoalState(curr):
    #         return path
    #     if curr in visited:
    #         continue
    #     visited.add(curr)
    #     for nbr, direction, cost in problem.getSuccessors(curr):
    #         if nbr not in visited:
    #             tmp = path + [direction]
    #             fringe.push((nbr, tmp))
    # return []

    fringe = Queue()
    start_state = problem.getStartState()
    fringe.push(start_state)
    fringe_path = dict()  # <state, optimal path from start to state>
    fringe_path[start_state] = []
    visited = set()
    while not fringe.isEmpty():
        curr = fringe.pop()
        path = fringe_path.pop(curr)
        if problem.isGoalState(curr):
            return path
        if curr in visited:
            continue
        visited.add(curr)
        for nbr, direction, cost in problem.getSuccessors(curr):
            if nbr not in visited and nbr not in fringe_path:
                fringe.push(nbr)
                fringe_path[nbr] = path + [direction]
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # fringe = PriorityQueue()
    # fringe.push((problem.getStartState(), []), 0)  # (state, the path from start to current state), cost as priority
    # visited = set()
    # while not fringe.isEmpty():
    #     curr, path = fringe.pop()
    #     if problem.isGoalState(curr):
    #         return path
    #     if curr in visited:  # for duplicates in fringe
    #         continue
    #     visited.add(curr)
    #     for nbr, direction, cost in problem.getSuccessors(curr):
    #         tmp = path + [direction]
    #         fringe.push((nbr, tmp), problem.getCostOfActions(tmp))
    # return []

    fringe = PriorityQueue()
    fringe.push(problem.getStartState(), 0)  # state, cost as priority
    fringe_path = dict()
    fringe_path[problem.getStartState()] = ([], 0)  # path, cost of path
    visited = set()
    while not fringe.isEmpty():
        curr = fringe.pop()
        path, path_cost = fringe_path.pop(curr)
        if problem.isGoalState(curr):
            return path
        if curr in visited:
            continue
        visited.add(curr)
        for nbr, direction, cost in problem.getSuccessors(curr):
            nbr_path = path + [direction]
            nbr_path_cost = problem.getCostOfActions(nbr_path)
            if nbr in fringe_path:
                if nbr_path_cost < fringe_path[nbr][1]:
                    fringe.update(nbr, nbr_path_cost)
                    fringe_path[nbr] = (nbr_path, nbr_path_cost)
            else:
                fringe.push(nbr, nbr_path_cost)
                fringe_path[nbr] = (nbr_path, nbr_path_cost)
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # fringe = PriorityQueue()
    # fringe.push((problem.getStartState(), []), 0)
    # visited = set()
    # while not fringe.isEmpty():
    #     curr, path = fringe.pop()
    #     if problem.isGoalState(curr):
    #         return path
    #     if curr in visited:  # for duplicates in fringe
    #         continue
    #     visited.add(curr)
    #     for nbr, direction, cost in problem.getSuccessors(curr):
    #         tmp = path + [direction]
    #         fringe.push((nbr, tmp), problem.getCostOfActions(tmp) + heuristic(nbr, problem))
    # return []

    fringe = PriorityQueue()
    fringe.push(problem.getStartState(), 0)  # state, cost as priority
    fringe_path = dict()
    fringe_path[problem.getStartState()] = ([], 0)  # path, cost of path
    visited = set()
    while not fringe.isEmpty():
        curr = fringe.pop()
        path, path_cost = fringe_path.pop(curr)
        if problem.isGoalState(curr):
            return path
        if curr in visited:
            continue
        visited.add(curr)
        for nbr, direction, cost in problem.getSuccessors(curr):
            nbr_path = path + [direction]
            nbr_path_cost = problem.getCostOfActions(nbr_path) + heuristic(nbr, problem)
            if nbr in fringe_path:
                if nbr_path_cost < fringe_path[nbr][1]:
                    fringe.update(nbr, nbr_path_cost)
                    fringe_path[nbr] = (nbr_path, nbr_path_cost)
            else:
                fringe.push(nbr, nbr_path_cost)
                fringe_path[nbr] = (nbr_path, nbr_path_cost)
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
