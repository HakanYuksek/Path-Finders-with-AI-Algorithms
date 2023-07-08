# -*- coding: utf-8 -*-
"""
Created on Sat Mar 14 19:13:15 2020

@author: USER
"""

# %% import Libraries
import numpy as np
import matplotlib.pyplot as plt
import math
from PIL import Image
import time

# %% methods

# This method calculates r value for the given point the image
def compute_R_value_in_RGB(map, pos):        
    map_inverted = 255 - map
    g_n = map_inverted[pos[0]][pos[1]][0] * 100
    return g_n

# This method calculates the distance between source and destination point
# By using Euclied Distance
def calculate_distance_by_Euclied(source, destination):
    dist = (source[0] - destination[0])**2 + (source[1]-destination[1])**2
    return dist
    

# This method computes the f(n) and return it for given position
def compute_point_of_state(map,position,destination,prev_g_n):
    g_n = compute_R_value_in_RGB(map,position) + prev_g_n
    h_n = calculate_distance_by_Euclied(position, destination)
    f_n = g_n + h_n
    return f_n,h_n,g_n

# This method finds the state which has the lowest point and return that state
def find_min(states):
    state = [1000000000,1000000000]
    min = 100000000000000000000000000
    min_h = 1000000000000000000000000
    index = 0
    min_index = 0
    for each in states:
        if(each[1] < min):
            min = each[1]
            state = each
            min_h = min
            min_index = index
        elif(each[1] == min and each[2] < min_h):
            min_h = each[2]
            state = each
            min_index = index
        index += 1
    print("min--->",state)
    return state,min_index


# %% Let's read the image from the file

map = np.array(Image.open("deneme4.png"))
plt.title("Harita ( Resim )")
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
back_up = []

# prev g_n can be used to compute current g_n
prev_g_n = compute_R_value_in_RGB(map,start)


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
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map, pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
    # To Right
    if((position[0] + 1) < len(map)):
        pos = [(position[0] + 1), position[1]]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map,pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
    # To Up
    if((position[1] - 1) >= 0):
        pos = [position[0] ,(position[1] - 1)]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map, pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
    # To Down
    if((position[1] + 1) < len(map[0])):
        pos = [position[0] ,(position[1] + 1)]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map, pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
    # To Upper Left
    if((position[0]-1)>=0 and (position[1] - 1) >= 0):
        pos = [(position[0]-1) ,(position[1] - 1)]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map,pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
    # To Upper Right
    if((position[0]-1)>=0 and (position[1] + 1) < len(map[0])):
        pos = [(position[0]-1) ,(position[1] + 1)]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map,pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
     # To Bottom Right
    if((position[0]+1) < len(map) and (position[1] + 1) < len(map[0])):
        pos = [(position[0]+1) ,(position[1] + 1)]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map,pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
     # To Bottom Left
    if((position[0]+1) < len(map) and (position[1] - 1) >= 0):
        pos = [(position[0]+1) ,(position[1] - 1)]
        if not pos in path and not pos in back_up:
            point, heuristic,g_n = compute_point_of_state(map, pos, target,prev_g_n)
            state = [pos,point, heuristic,g_n]
            points_of_states.append(state)
            back_up.append(state[0])
   
    selected_state,index = find_min(points_of_states)
    points_of_states.pop(index)
    position = selected_state[0]
    prev_g_n = selected_state[3]
    back_up.pop(index)
    path.append(position)

    if len(points_of_states) > max_number_of_elements_in_stack:
        max_number_of_elements_in_stack = len(points_of_states)
##################################################################
#   Alt kisim anlik hareketi görmek için yazildi
##################################################################
#    res_map = map.copy()
    
#    for each in path:
#       res_map[each[0]][each[1]] = 1
    
#    plt.title("Beyaz Çizgi Bulunan Path")
#    plt.imshow(res_map)
#    plt.show()

#    print("Path")
#    print("-------------------------------------")
#    print(path)
##################################################################

#%% We found the path, let's visualize it

end_time = time.time()

res_map = map.copy()

for each in path:
    res_map[each[0]][each[1]] = 1

plt.title("Beyaz Çizgi Bulunan Path")
plt.imshow(res_map)
plt.show()
print("-------------------------------------")
print("Stack ten çekilen eleman sayisi--->",len(path))
print("-------------------------------------")
print("Stackte maksimum eleman sayisi--->",max_number_of_elements_in_stack)
print("-------------------------------------")
print("Geçen Süre--->"+str((end_time-start_time))+" saniye")

