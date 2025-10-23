# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  from util import Stack

  # Stack will store tuples of (current_state, path_taken)
  stack = Stack()

  # Start state (Pacman's starting position)
  start_state = problem.getStartState()

  # Push the start state onto the stack with an empty path
  stack.push((start_state, []))

  # Keep track of visited states 
  visited = set()

  # Keep exploring until stack is empty
  while not stack.isEmpty():
      state, path = stack.pop()

      # If goal reached, return the moves Pacman will make
      if problem.isGoalState(state):
          return path

      # If we haven't already visited this , explore it
      if state not in visited:
          visited.add(state)

          # Get all possible moves (successors)
          for successor, action, stepCost in problem.getSuccessors(state):
              if successor not in visited:
                  # Add successor to stack with updated path
                  new_path = path + [action]
                  stack.push((successor, new_path))


  return []

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  "*** YOUR CODE HERE ***"
  from util import Queue  
  queue = Queue()

  start_state = problem.getStartState()

  # Queue holds (current_state, path_taken)
  queue.push((start_state, []))
  visited = set()

  while not queue.isEmpty():
      state, path = queue.pop()

      # Check if this is the goal
      if problem.isGoalState(state):
          return path

      # Explore this node if not already visited
      if state not in visited:
          visited.add(state)

          # Loop through all possible next states
          for successor, action, stepCost in problem.getSuccessors(state):
              if successor not in visited:
                  new_path = path + [action]
                  queue.push((successor, new_path))

  # If no path found (shouldn't happen in Pacman)
  return []

      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  from util import PriorityQueue  # PriorityQueue orders by cost
  pq = PriorityQueue()

  start_state = problem.getStartState()
  pq.push((start_state, [], 0), 0)  # (state, path, cost), priority = cost

  visited = {}  # stores the lowest known cost to reach each state

  while not pq.isEmpty():
      state, path, cost = pq.pop()

      # If we've reached the goal, return the path
      if problem.isGoalState(state):
          return path

      # Only expand this state if we found a cheaper path to it
      if state not in visited or cost < visited[state]:
          visited[state] = cost

          for successor, action, stepCost in problem.getSuccessors(state):
              new_cost = cost + stepCost
              new_path = path + [action]
              pq.push((successor, new_path, new_cost), new_cost)

  return []


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  from util import PriorityQueue

  # Priority queue stores (state, path, cost)
  frontier = PriorityQueue()
  start = problem.getStartState()
  frontier.push((start, [], 0), 0)  # f = g + h = 0 + heuristic(start)

  visited = {}  # maps state -> best cost found so far

  while not frontier.isEmpty():
      state, path, g_cost = frontier.pop()

      # Goal check
      if problem.isGoalState(state):
          return path

      # Only expand if this is the best known cost for this state
      if state not in visited or g_cost < visited[state]:
          visited[state] = g_cost

          # Explore successors
          for (next_state, action, stepCost) in problem.getSuccessors(state):
              new_g = g_cost + stepCost
              f = new_g + heuristic(next_state, problem)
              new_path = path + [action]
              frontier.push((next_state, new_path, new_g), f)

  return []

    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
