# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project.  You are free to use and extend these projects for educational
# purposes.  The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html
"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
from game import Actions

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
  #print "Start:", problem.getStartState()
  #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  #print "Start's successors:", problem.getSuccessors(problem.getStartState())
  startnode = problem.getStartState()
  frontier_list = util.Stack()
  frontier_list.push((startnode,'same',0))
  explored_list = util.Stack()
  
  while(True):
      if(frontier_list.isEmpty()):
          return [directions for _,directions,_ in explored_list.list]

      current_node = frontier_list.pop()

      if(problem.isGoalState(current_node[0])):
          explored_list.push(current_node)
          x = [directions for _,directions,_ in explored_list.list]
          return x[1:] #removing 1st item because it has 'same' direction

      if(current_node[0] not in [nodes for nodes,_,_ in explored_list.list]):
          explored_list.push(current_node)

      has_new_successor = False
      for action in problem.getSuccessors(current_node[0]):
          child_node = action
          if(child_node[0] not in [nodes for nodes,_,_ in explored_list.list] and child_node[0] not in [nodes for nodes,_,_ in frontier_list.list] and problem.getSuccessors(child_node[0])):
              frontier_list.push(child_node)
              has_new_successor = True

      if(not has_new_successor):
          explored_list.pop() # current node should go because all its successors are repeated or no
                              # successors
          next_item = frontier_list.list[-1]
          for node in reversed(explored_list.list):
              successors = problem.getSuccessors(node[0])
              if(next_item not in successors):
                  explored_list.pop()
              else:
                  break

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  "*** YOUR CODE HERE ***"
  startnode = problem.getStartState()
  frontier_list = util.PriorityQueue()
  frontier_list.push((startnode,'same',0),0)
  explored_list = None
  list_explored_nodes_multi_goals = [util.PriorityQueue()]
  depth = -1
  goal_index = 0
  explored_list = list_explored_nodes_multi_goals[goal_index]

  state_is_complex = False
  if(not isinstance(startnode,tuple)):
      state_is_complex = True
  def CheckNode(node,hascomplexstate):

    if(hascomplexstate):
          return node.blankLocation
    else:
          return node
  def CalculatePath(currentnode=None):

          all_nodes = []
          for ex_list in list_explored_nodes_multi_goals:
              for item in ex_list.heap:

                  all_nodes.append(item)
          
          if(not all_nodes):
              return []

          currentlvl = -1
          if(currentnode <> None):
              currentlvl = currentnode[2]
          tmp_reversed_path = None

          tmp_reversed_path = sorted(all_nodes,key=lambda x: x[0],reverse=True)

          tmp_queue = util.Queue()

          tmp_levels = list()

          for a,b in tmp_reversed_path:
             if(a not in tmp_levels):
                 if(a <= currentlvl):
                    tmp_levels.append(a)

          for i in range(0,len(tmp_levels)):
              
              lvl_i1_nodes = [b for a,b in tmp_reversed_path if(tmp_levels[i] == a)]

              if(currentnode == None):
                  if(i == 0 and len(lvl_i1_nodes) == 1):
                      lvl_i_nodes = [b for a,b in tmp_reversed_path if(tmp_levels[i] == a)]
                      tmp_queue.push(lvl_i_nodes[0])
                      continue
              elif(currentnode[0] not in [(item[0]) for item in tmp_queue.list]):
                  tmp_queue.push(currentnode)

              if(len(tmp_queue.list) == 0):
                  return []

              for node in lvl_i1_nodes:
                  last_node = -1
                  if(len(tmp_queue.list) == 1):
                      last_node = tmp_queue.list[0][0]
                  else:
                      last_node = tmp_queue.list[:-1][0][0]

                  direction = tmp_queue.list[0][1]
                  if(state_is_complex):
                      if(direction == 'up'): #row=y, col = x
                          dx = -1
                          dy = 0
                      if(direction == 'down'):
                          dx = 1
                          dy = 0
                      if(direction == 'right'):
                          dx = 0
                          dy = 1
                      elif(direction == 'left'):
                          dx = 0
                          dy = -1
                  else:
                      dx, dy = Actions.directionToVector(direction)

                  #x, y = int(node[0][0] + dx), int(node[0][1] + dy) #
                  #CheckNode
                  x, y = int(CheckNode(node[0],state_is_complex)[0] + dx), int(CheckNode(node[0],state_is_complex)[1] + dy)
                  if(CheckNode(last_node,state_is_complex) == (x,y)):
                     tmp_queue.push(node)
                     break

          res = [(item[1]) for item in tmp_queue.list]
          return res[1:]



  while(True):
      if(frontier_list.isEmpty()):
          #return [(item[1][1]) for item in explored_list.heap]
          if(len(explored_list.heap) == 1 or not explored_list.heap):
              return CalculatePath()
          else:
              return CalculatePath((explored_list.heap[-1][1][0],explored_list.heap[-1][1][1],explored_list.heap[-1][0]))
      
      if(frontier_list.heap):
          depth = frontier_list.heap[0][0]

      current_node = frontier_list.pop()

      if(CheckNode(current_node[0],state_is_complex) not in [CheckNode(item[1][0],state_is_complex) for item in explored_list.heap]):
          explored_list.push(current_node, depth)


      for action in problem.getSuccessors(current_node[0]):
          child_node = action
          if(CheckNode(child_node[0],state_is_complex) not in [CheckNode(item[1][0],state_is_complex) for item in explored_list.heap] and CheckNode(child_node[0],state_is_complex) not in [CheckNode(item[1][0],state_is_complex) for item in frontier_list.heap]):
              is_temp_goal = False
              is_final_goal = False

              if(isinstance(problem.isGoalState(child_node[0]),bool)):
                  is_final_goal = problem.isGoalState(child_node[0])
              else:

                  is_temp_goal,is_final_goal,goal_index = problem.GetIsGoalStateResult(CheckNode(child_node[0],state_is_complex))

              if(is_temp_goal and not is_final_goal):
                  if(len(list_explored_nodes_multi_goals) == goal_index):
                      frontier_list.heap = []
                      frontier_list.push(child_node, depth + 1)
                      list_explored_nodes_multi_goals.append(util.PriorityQueue())
                      explored_list = list_explored_nodes_multi_goals[goal_index]
                      break
                    

              if(is_final_goal):
                  explored_list.push(child_node, depth + 1)

                  return CalculatePath((CheckNode(child_node[0],state_is_complex),child_node[1],depth + 1))

              frontier_list.push(child_node, depth + 1)

      successors = problem.getSuccessors(current_node[0])

      successors_in_frontier = [CheckNode(successor[0],state_is_complex) for successor in successors if(CheckNode(successor[0],state_is_complex) in [CheckNode(item[1][0],state_is_complex) for item in frontier_list.heap])]

      if(not successors_in_frontier):
          explored_list.heap.remove((depth,current_node))

def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  def CalculatePath(currentnode=None):
          if(not explored_list.heap):
              return []
          tmp_reversed_path = None

          tmp_reversed_path = sorted(explored_list.heap,key=lambda x: x[0],reverse=True)

          tmp_queue = util.Queue()

          tmp_levels = list()

          for a,b in tmp_reversed_path:
             if(a not in tmp_levels):
                 tmp_levels.append(a)

          for i in range(0,len(tmp_levels)):
              
              lvl_i1_nodes = [b for a,b in tmp_reversed_path if(tmp_levels[i] == a)]

              if(currentnode == None):
                  if(i == 0 and len(lvl_i1_nodes) == 1):
                      lvl_i_nodes = [b for a,b in tmp_reversed_path if(tmp_levels[i] == a)]
                      tmp_queue.push(lvl_i_nodes[0])
                      continue
              elif(currentnode[0] not in [(item[0]) for item in tmp_queue.list]):
                  tmp_queue.push(currentnode)

              if(len(tmp_queue.list) == 0):
                  return []

              for node in lvl_i1_nodes:
                  last_node = -1
                  if(len(tmp_queue.list) == 1):
                      last_node = tmp_queue.list[0][0]
                  else:
                      last_node = tmp_queue.list[:-1][0][0]

                  if(last_node in [(x[0]) for x in problem.getSuccessors(node[0])]):
                      tmp_queue.push(node)
                      break

          return [(item[1]) for item in tmp_queue.list]

  startnode = problem.getStartState()
  frontier_list = util.PriorityQueue()

  frontier_list.push((startnode,'same',0),0)
  explored_list = util.PriorityQueue()
  depth = -1
  while(True):
      if(frontier_list.isEmpty()):
          return [(item[1][1]) for item in explored_list.heap]
      
      if(frontier_list.heap):
          depth = frontier_list.heap[0][1][2]

      current_node = frontier_list.pop()

      if(current_node[0] not in [(item[1][0]) for item in explored_list.heap]):
          if(current_node[0] <> startnode):
             explored_list.push(current_node, depth)

      current_node_path = CalculatePath(current_node)

      for action in problem.getSuccessors(current_node[0]):
          child_node = action
          if(child_node[0] == startnode):
              continue
          
          cost = problem.getCostOfActions(current_node_path)

          if(child_node[0] not in [(item[1][0]) for item in explored_list.heap] and child_node[0] not in [(item[1][0]) for item in frontier_list.heap]):
              if(problem.isGoalState(child_node[0])):
                  explored_list.push(child_node, depth + 1)
                  current_node_path.append(child_node[1])
                  return current_node_path
                  
              frontier_list.push((child_node[0],child_node[1],depth + 1),cost)
              
          elif(child_node[0] in [(item[1][0]) for item in frontier_list.heap]):
               front_node = [item for item in frontier_list.heap if(item[0] > cost and child_node[0] == item[1][0])]
               if(front_node):
                   frontier_list.heap.remove(front_node[0])
                   frontier_list.push((child_node[0],child_node[1],front_node[0][1][2]),cost)

      successors = problem.getSuccessors(current_node[0])
      successors_in_frontier = [(successor[0]) for successor in successors if(successor[0] in [(item[1][0]) for item in frontier_list.heap])]

      if(not successors_in_frontier):
          explored_list.heap.remove((depth,current_node))
            
def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  def CalculatePath(currentnode=None):

          all_nodes = []
          for ex_list in list_explored_nodes_multi_goals:
              for item in ex_list.heap:
                  if(state_has_grid):
                      all_nodes.append([item[0],(item[1][0][0],item[1][1],item[1][2])])
                  else:
                      all_nodes.append(item)
          
          if(not all_nodes):
              return []

          if(state_has_grid):
              currentnode = (currentnode[0][0],currentnode[1],currentnode[2])

          currentlvl = -1
          if(currentnode <> None):
              currentlvl = currentnode[2]
          tmp_reversed_path = None

          tmp_reversed_path = sorted(all_nodes,key=lambda x: x[0],reverse=True)

          tmp_queue = util.Queue()

          tmp_levels = list()

          for a,b in tmp_reversed_path:
             if(a not in tmp_levels):
                 if(a <= currentlvl):
                    tmp_levels.append(a)

          for i in range(0,len(tmp_levels)):
              
              lvl_i1_nodes = [b for a,b in tmp_reversed_path if(tmp_levels[i] == a)]

              if(currentnode == None):
                  if(i == 0 and len(lvl_i1_nodes) == 1):
                      lvl_i_nodes = [b for a,b in tmp_reversed_path if(tmp_levels[i] == a)]
                      tmp_queue.push(lvl_i_nodes[0])
                      continue
              elif(currentnode[0] not in [(item[0]) for item in tmp_queue.list]):
                  tmp_queue.push(currentnode)

              if(len(tmp_queue.list) == 0):
                  return []

              for node in lvl_i1_nodes:
                  last_node = -1
                  if(len(tmp_queue.list) == 1):
                      last_node = tmp_queue.list[0][0]
                  else:
                      last_node = tmp_queue.list[:-1][0][0]

                  dx, dy = Actions.directionToVector(tmp_queue.list[0][1])
                  x, y = int(node[0][0] + dx), int(node[0][1] + dy)
                  if(last_node == (x,y)):
                     tmp_queue.push(node)
                     break

          res = [(item[1]) for item in tmp_queue.list]
          return res

  startnode = problem.getStartState()
  frontier_list = util.PriorityQueue()

  frontier_list.push((startnode,'same',0),0)
  explored_list = None
  list_explored_nodes_multi_goals = [util.PriorityQueue()]
  goal_index = 0
  explored_list = list_explored_nodes_multi_goals[goal_index]
  depth = -1

  state_has_grid = False
  if(not isinstance(startnode[1],int)):
      state_has_grid = True

  def CheckNode(node,hasgrid):

      if(hasgrid):
          return node[0]
      else:
          return node

  while(True):
      if(frontier_list.isEmpty()):
          #return [(item[1][1]) for item in explored_list.heap]
          if(len(explored_list.heap) == 1 or not explored_list.heap):
              return CalculatePath()
          else:
              return CalculatePath(explored_list.heap[-1][1])
      
      if(frontier_list.heap):
          depth = frontier_list.heap[0][1][2]

      current_node = frontier_list.pop()

      if(CheckNode(current_node[0],state_has_grid) not in [CheckNode(item[1][0],state_has_grid) for item in explored_list.heap]):
          if(CheckNode(current_node[0],state_has_grid) <> CheckNode(startnode,state_has_grid)):
             explored_list.push(current_node, depth)


      current_node_path = CalculatePath(current_node)

      for action in problem.getSuccessors(current_node[0]):
          
          if(CheckNode(action[0],state_has_grid) == CheckNode(startnode,state_has_grid)):
              continue
          
          cost = problem.getCostOfActions(current_node_path) + heuristic(current_node[0],problem)

          if(CheckNode(action[0],state_has_grid) not in [CheckNode(item[1][0],state_has_grid) for item in explored_list.heap] and CheckNode(action[0],state_has_grid) not in [CheckNode(item[1][0],state_has_grid) for item in frontier_list.heap]):
              is_temp_goal = False
              is_final_goal = False

              if(isinstance(problem.isGoalState(action[0]),bool)):
                  if(not state_has_grid):
                      if(problem.isGoalState(action[0])):
                           explored_list.push(action, depth + 1)
                           current_node_path.append(action[1])
                           return current_node_path
                  else:
                      #Q7
                      if(problem.isGoalState(action[0])):
                          explored_list.push(action, depth + 1)
                          current_node_path.append(action[1])
                          return current_node_path
                      else:
                          if(problem.goalslistchanged):
                              problem.goalslistchanged = False
                              goal_index = problem.goalsvisitedcount
                              frontier_list.heap = []
                              frontier_list.push((action[0],action[1],depth + 1),cost)
                              list_explored_nodes_multi_goals.append(util.PriorityQueue())
                              explored_list = list_explored_nodes_multi_goals[goal_index]
                              break
              else:
                  #Q6
                  is_temp_goal,is_final_goal,goal_index = problem._is_goal_result[CheckNode(action[0],state_has_grid)]

                  if(is_temp_goal and not is_final_goal):
                    frontier_list.heap = []
                    
                    frontier_list.push((action[0],action[1],depth + 1),cost)
                    list_explored_nodes_multi_goals.append(util.PriorityQueue())
                    explored_list = list_explored_nodes_multi_goals[goal_index]

                    break
                  
                  if(is_final_goal):
                     explored_list.push(action, depth + 1)
                     current_node_path.append(action[1])
                     return current_node_path
       
              frontier_list.push((action[0],action[1],depth + 1),cost)
           
          
          elif(CheckNode(action[0],state_has_grid) in [CheckNode(item[1][0],state_has_grid) for item in frontier_list.heap]):

               front_node = [item for item in frontier_list.heap if(item[0] > cost and CheckNode(action[0],state_has_grid) == CheckNode(item[1][0],state_has_grid))]
               if(front_node):
                   frontier_list.heap.remove(front_node[0])
                   frontier_list.push((action[0],action[1],depth + 1),cost)
                   

      successors = problem.getSuccessors(current_node[0])
      successors_in_frontier = [CheckNode(successor[0],state_has_grid) for successor in successors if(CheckNode(successor[0],state_has_grid) in [CheckNode(item[1][0],state_has_grid) for item in frontier_list.heap])]

      if(not successors_in_frontier):
          explored_list.heap.remove((depth,current_node))
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
