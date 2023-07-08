# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 17:46:59 2020

@author: USER
"""

# %% import Libraries
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import time

# %% methods

# This method calculates the distance between source and destination point
# By using Euclied Distance
def calculate_distance_by_Euclied(source, destination):
    dist = (source[0] - destination[0])**2 + (source[1]-destination[1])**2
    return dist

# This method finds the state which has the lowest point and return that state
def find_min(states):
    state = [1000,1000]
    min = 1000000000000000000000000000000
    min_h = 10000000000000000000000000000
    for each in states:
        if(each[1] < min):
            min = each[1]
            state = each
            min_h = min
    print("min--->",state)
    return state


# %% Let's read the image from the file

map = np.array(Image.open("deneme4.png"))

plt.title("Harita (Resim)")
plt.imshow(map)
plt.show()

# %% Now let's begin the algorithm

# Firstly we need a start position
start = [0,0]

# Of course we also need a goal position
target = [(len(map)-1),(len(map[0])-1)]

# %% Actual algorithm part

# We need a list to hold point of each state
points_of_states = []

# We hold our position
position = start.copy()

# We should hold the result path
path = [position]

# We show the max number of elements in the stack
max_number_of_elements_in_stack = 1

# We have to calculate the algorithm's runtime
start_time = time.time()

while((position[0] != target[0]) or (position[1] != target[1])):
    
    # To Left
    if((position[0]-1) >= 0):
        pos = [(position[0] - 1), position[1]]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
    # To Right
    if((position[0] + 1) < len(map)):
        pos = [(position[0] + 1), position[1]]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
    # To Up
    if((position[1] - 1) >= 0):
        pos = [position[0] ,(position[1] - 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
    # To Down
    if((position[1] + 1) < len(map[0])):
        pos = [position[0] ,(position[1] + 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
    # To Upper Left
    if((position[0]-1)>=0 and (position[1] - 1) >= 0):
        pos = [(position[0]-1) ,(position[1] - 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
    # To Upper Right
    if((position[0]-1)>=0 and (position[1] + 1) < len(map[0])):
        pos = [(position[0]-1) ,(position[1] + 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
     # To Bottom Right
    if((position[0]+1) < len(map) and (position[1] + 1) < len(map[0])):
        pos = [(position[0]+1) ,(position[1] + 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
     # To Bottom Left
    if((position[0]+1) < len(map) and (position[1] - 1) >= 0):
        pos = [(position[0]+1) ,(position[1] - 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [pos,point]
        points_of_states.append(state)
        
    selected_state = find_min(points_of_states)
    points_of_states.remove(selected_state)
    position = selected_state[0]
    
    path.append(position)

    if len(points_of_states) > max_number_of_elements_in_stack:
        max_number_of_elements_in_stack = len(points_of_states)
#    print("Path")
#    print("-------------------------------------")
#    print(path)
#%% We found the path, let's visualize it

end_time = time.time()

res_map = map.copy()

for each in path:
    res_map[each[0]][each[1]] = 1

plt.title("Siyah Çizgi Bulunan Path")
plt.imshow(res_map)
plt.show()

print("-------------------------------------")
print("Stack ten çekilen eleman sayisi--->",len(path))
print("-------------------------------------")
print("Stackte maksimum eleman sayisi--->",max_number_of_elements_in_stack)
print("-------------------------------------")
print("Geçen Süre--->"+str((end_time-start_time))+" saniye")
