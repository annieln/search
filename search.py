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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
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
    """
    print("")
    print("Start:", problem.getStartState())
    print("")
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("")
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    print("")
    """
    
    start = nodeState = problem.getStartState()
    fringe = util.Stack()
    closed = []
    path = []
    parents = {}

    while True:
        if fringe.isEmpty():
            closed.append(start)
            successors = problem.getSuccessors(start)
            for successor in successors:
                fringe.push(successor)
                parents[successor[0]] = [start, successor[1]]
        else:
            node = fringe.pop()
            nodeState = node[0]
            closed.append(nodeState)
            #print("Current fringe: ", fringe.list)

            if problem.isGoalState(nodeState):
                break
            
            successors = problem.getSuccessors(nodeState)
            #print("Successors: ", successors)
            for successor in successors:
                if successor[0] not in closed:
                    fringe.push(successor)
                    parents[successor[0]] = [nodeState, successor[1]]
            
            #print("Updated fringe: ", fringe.list)


            #if nodeState not in closed.list:
             #   closed.push(nodeState)
                #print("New closed list: ", closed.list)

    while nodeState != start:
        pState, pDir = parents[nodeState]
        path.append(pDir)
        nodeState = pState

    path.reverse()

    return path

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = nodeState = problem.getStartState()
    fringe = util.Queue()
    closed = []
    visit = []
    path = []
    parents = {}

    while True:
        if fringe.isEmpty():
            closed.append(start)
            successors = problem.getSuccessors(start)
            for successor in successors:
                fringe.push(successor)
                visit.append(successor[0])
                parents[successor[0]] = [start, successor[1]]
        else:
            node = fringe.pop()
            nodeState = node[0]

            if problem.isGoalState(nodeState):
                break
            else:
                closed.append(nodeState)
                successors = problem.getSuccessors(nodeState)
                for successor in successors:
                    if successor[0] not in closed and successor[0] not in visit:
                        fringe.push(successor)
                        visit.append(successor[0])
                        parents[successor[0]] = [nodeState, successor[1]]

    while nodeState != start:
        pState, pDir = parents[nodeState]
        path.append(pDir)
        nodeState = pState

    path.reverse()

    return path

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start = state = problem.getStartState()
    fringe = util.PriorityQueue()
    closed = []
    path = []
    visit = {}
    parents = {}

    closed.append(start)
    successors = problem.getSuccessors(start)
    for successor in successors:
        state, action, cost = successor
        fringe.push((state, action, cost), cost)
        parents[state] = [start, action]
        visit[state] = cost

    while not fringe.isEmpty():
        node = fringe.pop()
        state, action, cost = node

        if problem.isGoalState(state):
            break
        else:
            if state not in closed:
                closed.append(state)
                successors = problem.getSuccessors(state)
                
                for successor in successors:
                    childState, childAction, childCost = successor
                    if childState in closed:
                        continue # if we already explored this node, we don't need to add it to visit again

                    fringe.update((childState, childAction, cost + childCost), cost + childCost)
                    if childState in visit:
                        if cost + childCost < visit[childState]:
                            visit[childState] = cost + childCost
                            parents[childState] = [state, childAction]
                    else:
                        visit[childState] = cost + childCost
                        parents[childState] = [state, childAction]
 
    while state != start:
        pState, pAction = parents[state]
        path.append(pAction)
        state = pState
    
    path.reverse()

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start = state = problem.getStartState()
    closed = []
    path = []
    visit = {}
    parents = {}

    def f(n):
        return visit[n] + heuristic(n, problem=problem)
    
    fringe = util.PriorityQueueWithFunction(f)

    closed.append(start)
    successors = problem.getSuccessors(start)
    for successor in successors:
        state, action, cost = successor
        parents[state] = [start, action]
        visit[state] = cost
        fringe.push(state)

    #print(closed)
    #print(fringe.heap)
    #print("Current fringe: ", fringe.heap)
    while not fringe.isEmpty():
        state = fringe.pop()
        #print("Current state: ", state)

        if problem.isGoalState(state):
            break
        else:
            if state not in closed:
                closed.append(state)
                successors = problem.getSuccessors(state)
                #print("These are the successors: ", successors)
                
                for successor in successors:
                    #print("Current successor: ", successor)
                    childState, childAction, childCost = successor
                    if childState in closed:
                        continue # if we already explored this node, we don't need to add it to visit again

                    if childState in visit:
                        #print("New cost: ", visit[state] + childCost)
                        #print("Old cost: ", visit[childState])
                        if visit[state] + childCost < visit[childState]:
                            visit[childState] = visit[state] + childCost
                            parents[childState] = [state, childAction]
                            fringe.push(childState)
                    else:
                        visit[childState] = visit[state] + childCost
                        parents[childState] = [state, childAction]
                        fringe.push(childState)

                #print("")
                #print("Current fringe: ", fringe.heap)
 
    while state != start:
        pState, pAction = parents[state]
        path.append(pAction)
        state = pState
    
    path.reverse()

    return path


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
