# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.


import inspect
import sys
import random

from queue import PriorityQueue, Queue

def raiseNotDefined():
    fileName = inspect.stack()[1][1]
    line = inspect.stack()[1][2]
    method = inspect.stack()[1][3]

    print("*** Method not implemented: %s at line %s of %s" % (method, line, fileName))
    sys.exit(1)


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
        pass

    def isGoalState(self, state):
        """
        state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        pass

    def getSuccessors(self, state):
        """
        state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        pass

    def getCostOfActions(self, actions):
        """
        actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        pass

class Node:
    def __init__(self, state, action, cost):
        self.state = state
        self.action = action
        self.cost = cost
    
    def __lt__(self,other):
        return self.cost < other.cost

def random_search(problem):
    """
    Search the nodes in the search tree randomly.

    Your search algorithm needs to return a list of actions that reaches the goal.
    Make sure to implement a graph search algorithm.

    This random_search function is just example not a solution.
    You can write your code by examining this function
    """
    start = problem.getStartState()
    node = [(start, "", 0)]   # class is better
    frontier = [node]

    explored = set()
    while frontier:
        node = random.choice(frontier)
        state = node[-1][0]
        if problem.isGoalState(state):
            return [x[1] for x in node][1:]

        if state not in explored:
            explored.add(state)

            for successor in problem.getSuccessors(state):
                if successor[0] not in explored:
                    parent = node[:]
                    parent.append(successor)
                    frontier.append(parent)

    return []


def depth_first_search(problem):
    """Search the deepest nodes in the search tree first."""

    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    node = [Node(start, "", 0)]   # class is better
    frontier = [node]

    explored = set()

    while frontier:
        node = frontier.pop()
        state = node[-1].state
        if problem.isGoalState(state):
            return [x.action for x in node][1:]

        if state not in explored:
            explored.add(state)
            for successor in problem.getSuccessors(state):
                if successor[0] not in explored:
                    parent = node[:]
                    parent.append(Node(successor[0], successor[1], successor[2]))
                    frontier.append(parent)
    return []

    # raiseNotDefined()


def breadth_first_search(problem):
    """Search the shallowest nodes in the search tree first."""

    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    node = [Node(start, "", 0)]   # class is better
    frontier = Queue()
    frontier.put(node)

    explored = set()
    explored.add(start)

    while frontier:
        node = frontier.get()
        state = node[-1].state
        if problem.isGoalState(state):
            return [x.action for x in node][1:]

        for successor in problem.getSuccessors(state):
            if successor[0] not in explored:
                explored.add(successor[0])
                parent = node[:]
                parent.append(Node(successor[0], successor[1], successor[2]))
                frontier.put(parent)
    return []
    # raiseNotDefined()


def uniform_cost_search(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    node = [Node(start, "", 0)]   # class is better
    frontier = PriorityQueue()
    frontier.put(node)

    explored = dict()
    explored[start] = 0

    while frontier:
        node = frontier.get()
        state = node[-1].state
        if problem.isGoalState(state):
            return [x.action for x in node][1:]

        for successor in problem.getSuccessors(state):
            successor_cost = node[-1].cost + successor[2]
            if successor[0] not in explored or explored[successor[0]] > successor_cost:
                explored[successor[0]] = successor_cost
                parent = node[:]
                parent.append(Node(successor[0], successor[1], successor_cost))
                frontier.put(parent)

    return []
    # raiseNotDefined()

def heuristic_number_of_misplaced_tiles(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem. This heuristic is trivial.
    """
    "*** YOUR CODE HERE ***"
    current = 1
    count = 0
    for i in range(3):
        for j in range(3):
            if current % 9 != state.cells[i][j]:
                count += 1
            current += 1
    
    return count

def heuristic_manhattan(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem. This heuristic is trivial.
    """
    "*** YOUR CODE HERE ***"
    return 0


def aStar_search(problem, heuristic=heuristic_number_of_misplaced_tiles):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    node = [Node(start, "", heuristic(start))]   # class is better
    frontier = PriorityQueue()
    frontier.put(node)

    explored = dict()
    explored[start] = 0

    while frontier:
        node = frontier.get()
        state = node[-1].state
        current_heuristic = heuristic(state)
        if problem.isGoalState(state):
            return [x.action for x in node][1:]

        for successor in problem.getSuccessors(state):
            successor_cost = node[-1].cost + successor[2] - current_heuristic + heuristic(successor[0])
            if successor[0] not in explored or explored[successor[0]] > successor_cost:
                explored[successor[0]] = successor_cost
                parent = node[:]
                parent.append(Node(successor[0], successor[1], successor_cost))
                frontier.put(parent)

    return []
    # raiseNotDefined()


# Abbreviations
rand = random_search
bfs = breadth_first_search
dfs = depth_first_search
astar = aStar_search
ucs = uniform_cost_search
