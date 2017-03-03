#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
import math
import numpy as np
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

WALL_NUMBER = 10000
THE_OTHERS_NUMBER = 5000

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    man_distance = 0
    for box in list(state.boxes.keys()):
      min_distance = math.inf
      #If no restrictions, directly sum every smallest distance from any box to any storage
      if state.restrictions ==  None:
        for storage in state.storage:
          temp = abs(storage[0] - box[0])+abs(storage[1] - box[1])
          if(temp < min_distance):
            min_distance = temp
        man_distance+=min_distance
        continue
      #With restrictions, the index of restrictions should match the box index
      for restriction in state.restrictions[state.boxes[box]]:
        temp = abs(restriction[0] - box[0])+abs(restriction[1] - box[1])
        if(temp < min_distance):
          min_distance = temp
      man_distance+=min_distance
    return man_distance

# See tips on how my algorithm works
def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    #Create the map before calculating the heuristic value
    #A state can only have a fixed map, once it the map is settled, no need to create it, only create it once
    if(state.parent is None):
      create_map(state)
    #One restriction corresponds to one map_index, all maps are store in map_lst for future access and they are accessed
    #by map_index, global variable declared in create_map() function. Map without restriction has one dimension by 
    #default. (Here dimensions is just how many maps are stored in a list)
    for map_index in range(dimensions):  
        #Recursively expand the map staring from storage point using BFS, and the cost for one movement is just one
        #open_lst is a list of list used to store points in the map that are going to be explored the first index 
        #in open_lst is to denote per restriction-related map, the second index is to denote element to be explored
        while(len(open_lst[map_index])!=0):
            #Alwasy append new point to be explored at the end of the list
            expand(open_lst[map_index][0], map_index)
            #Remove the first element in the list to make the while loop stop
            del open_lst[map_index][0]
    count = 0
    if(state.restrictions):
        #For restriction-related map list, we need to sum every single map's cost. And each map's cost is calculated by
        #summing up all the BFS value from the box to corresponding storage point where that box can be put into. 
        for res_index in range(dimensions):
            in_box = [box for box, box_index in state.boxes.items() if res_index==box_index]
            for box in in_box:
                count+=map_lst[res_index][box]
    else:
        #Handingling no restriction map
        for box in list(state.boxes.keys()):
            count+=map_lst[0][box]
    return count

def create_map(state):
    #Declare global variable for access across functions
    global open_lst
    global map_lst
    global ex_lst
    global map_width
    global map_height
    global dimensions
    map_width = state.width
    map_height = state.height
    if state.restrictions:
        dimensions = len(state.restrictions)
    else:
        dimensions = 1
    #open_lst used for storing point in the map that is going to be explored, first index is to position 
    #the restriction-bound map
    open_lst = []
    #map_lst is to store all maps, can be accessed directly by index to get the map
    map_lst = []
    #ex_lst is to store those points in the map that has been explored, no need to explored once more, ex_lst
    #is the same structure as open_lst
    ex_lst = []
    for i in range(dimensions):
        #Populate the map, storage point to be 0, wall to be WALL_NUMBER(10000) and the rest to be 
        #THE_OTHER_NUMBER(5000)
        sokoban_map = np.full((map_width, map_height),THE_OTHERS_NUMBER)
        ex_sublst = []
        open_sublst = []
        for obs in state.obstacles:
            sokoban_map[obs] = WALL_NUMBER
            ex_sublst.append(obs)
        #Every map with corresponding restriction should be individually computed
        if(state.restrictions):
            for storage_point in state.restrictions[i]:
                #Map is not correclty populated in Problem 21, this is to avoid error from that part
                try:
                    sokoban_map[storage_point] = 0
                except:
                    pass
                open_sublst.append(storage_point)
                ex_sublst.append(storage_point)
        else:
            for storage_point in list(state.storage.keys()):
                sokoban_map[storage_point] = 0
                open_sublst.append(storage_point)
                ex_sublst.append(storage_point)     
        open_lst.append(open_sublst)  
        ex_lst.append(ex_sublst)
        map_lst.append(sokoban_map)
    
def expand(e_point,index):
    #expansion is map-specific such that only one recursion will take place within one call to this function
    #four neighbors around the e_point are updated and if necessary, appended at the end of open_lst
    ex_lst[index].append(e_point)
    left_neighbor = (e_point[0]-1,e_point[1])
    right_neighbor = (e_point[0]+1,e_point[1])
    up_neighbor = (e_point[0],e_point[1]-1)
    down_neighbor = (e_point[0],e_point[1]+1)
    sokoban_map = map_lst[index]
    # Calculate the value of every neighbor
    if(e_point[0] in range(2,map_width) and left_neighbor not in ex_lst[index]):
        #If the wall are two step away, it means it is impossible for a box to come in from that direction
        #Then put a big number(WALL_NUMBER) to indicate the impossibility there
        if(sokoban_map[(e_point[0]-2,e_point[1])]==WALL_NUMBER):
            sokoban_map[left_neighbor]=min(sokoban_map[left_neighbor],WALL_NUMBER)
        #else by definition of BFS, the value of e_point should by update by min of the neighbor and itself plus ones
        else:
            sokoban_map[left_neighbor]=min(sokoban_map[left_neighbor], \
              sokoban_map[(e_point[0],e_point[1])]+1)
            if(left_neighbor not in open_lst[index]):
                open_lst[index].append(left_neighbor)
    if(e_point[0] in range(0,map_width-2) and right_neighbor not in ex_lst[index]):
        if(sokoban_map[(e_point[0]+2,e_point[1])]==WALL_NUMBER):
            sokoban_map[right_neighbor]=min(sokoban_map[right_neighbor], WALL_NUMBER)
        else:
            sokoban_map[right_neighbor]=min(sokoban_map[right_neighbor], \
              sokoban_map[(e_point[0],e_point[1])]+1)
            if(right_neighbor not in open_lst[index]):
                open_lst[index].append(right_neighbor)
    if(e_point[1] in range(2,map_height) and up_neighbor not in ex_lst[index]):
        # This is to avoid Problem 21 error where the map is not correctly populated
        try:
            if(sokoban_map[(e_point[0],e_point[1]-2)]==WALL_NUMBER):        
                sokoban_map[up_neighbor]=min(sokoban_map[up_neighbor], WALL_NUMBER)
            else:
                sokoban_map[up_neighbor]=min(sokoban_map[up_neighbor], \
                  sokoban_map[(e_point[0],e_point[1])]+1)
                if(up_neighbor not in open_lst[index]):
                    open_lst[index].append(up_neighbor)
        except:
            pass
    if(e_point[1] in range(0,map_height-2) and down_neighbor not in ex_lst[index]):
        if(sokoban_map[(e_point[0],e_point[1]+2)]==WALL_NUMBER):        
            sokoban_map[down_neighbor]=min(sokoban_map[down_neighbor], WALL_NUMBER)
        else:
            sokoban_map[down_neighbor]=min(sokoban_map[down_neighbor], \
              sokoban_map[(e_point[0],e_point[1])]+1) 
            if(down_neighbor not in open_lst[index]):
                open_lst[index].append(down_neighbor)
    map_lst[index] = sokoban_map
    return
  
def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy b if state.restritions:est-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    #Set mode for the Seach Engine
    se = SearchEngine(strategy='best_first',cc_level='full')
    #Init the search engine
    se.init_search(initial_state, sokoban_goal_state, heur_fn=heur_fn)
    start_time = os.times()[0]
    #start searching with the time bound
    new_state = se.search(timebound=timebound)
    result = new_state
    #keep looping until no better solution can be found or run out of time
    while new_state:
      cost =(new_state.gval, math.inf, math.inf)
      result = new_state
      remaining_time = timebound - (os.times()[0] - start_time)
      new_state = se.search(timebound=remaining_time, costbound=cost)
    return result

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT        
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    #Implmentation are almost the same as anytime_gbfs except for we need a wrapper function here
    se = SearchEngine(strategy='custom',cc_level='full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn=heur_fn,fval_function=wrapped_fval_function)
    start_time = os.times()[0]
    new_state = se.search(timebound=timebound)
    result = new_state
    while new_state:
      cost =(math.inf, math.inf, new_state.gval+heur_manhattan_distance(new_state))
      result = new_state
      remaining_time = timebound - (os.times()[0] - start_time)
      new_state = se.search(timebound=remaining_time, costbound=cost)
    return result

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 


