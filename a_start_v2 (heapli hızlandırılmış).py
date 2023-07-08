# -*- coding: utf-8 -*-
"""
Created on Thu Feb 27 19:12:41 2020

@author: USER
"""

# %% import Libraries
import numpy as np
import matplotlib.pyplot as plt
import time 
from PIL import Image

# %% class

class HeapTree:
    
    def __init__(self):
        self.nodes =  []
        self.heapSize = 0
    # Up heap is necessary operation after the adding new elements to the heap
    def upHeap(self):
        i = self.heapSize - 1
        flag = True
        while i > 0 and flag:
            # if i is even, we should look at the (i -2 )/2
            if i % 2 == 0:   
                k = (i - 2) / 2
            # if i is odd, we should look at the (i-1)/2
            else:
                k = (i - 1) / 2
            k = int(k)
            if k >= 0 and self.nodes[k][0] > self.nodes[i][0]:
                tmp = self.nodes[k]
                self.nodes[k] = self.nodes[i]
                self.nodes[i] = tmp
                i = k
                
            else:
                flag = False
    # Down heap is necessary operation after the removing an elements from the heap
    def downHeap(self):
        i = 0
        flag = True
        while i < self.heapSize and flag:
            left_child_index = 2 * i + 1
            right_child_index = 2 * i + 2
            if right_child_index < self.heapSize:
                if self.nodes[right_child_index][0] < self.nodes[left_child_index][0]:
                    min_child = right_child_index
                else:
                    min_child = left_child_index
            elif left_child_index < self.heapSize:
                min_child = left_child_index
            else:
                min_child = i
            
            if self.nodes[i][0] > self.nodes[min_child][0]:
                tmp = self.nodes[i]
                self.nodes[i] = self.nodes[min_child]
                self.nodes[min_child] = tmp
                i = min_child
            else:
                flag = False
    # This method has to be used for adding new elements to the heap tree            
    def addElement(self,val):
        self.nodes.append(val)
        self.heapSize += 1
        self.upHeap()
    # This method has to be used for removing an element from the heap
    # This method returns root of the tree which has the minimum value
    def removeElement(self):
        if self.heapSize > 0:
            # Swapping between first and last element in the heap
            min_value = self.nodes[0]
            self.nodes[0] = self.nodes[self.heapSize-1]
            self.nodes[self.heapSize-1] = min_value
            self.nodes.pop((self.heapSize-1))
            # We should decrease the heap size
            self.heapSize -= 1
            self.downHeap()

            return min_value
        else:
            return None
    # This method is written to write the elements of the heap tree
    def writeElements(self):
        list1 = []
        for i in range(self.heapSize):
            list1.append(self.nodes[i])
        print(list1)
# %% methods

# This method calculates r value for the given point the image
def compute_R_value_in_RGB(map, pos):
    map_inverted = 255 - map
    g_n = map_inverted[pos[0]][pos[1]][0] *100
    return g_n

# This method calculates the distance between source and destination point
# By using Euclied Distance
def calculate_distance_by_Euclied(source, destination):
    dist = (source[0] - destination[0])**2 + (source[1]-destination[1])**2
    return dist


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

# We need a heap to hold the state's point
heap = HeapTree()
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
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
    # To Right
    if((position[0] + 1) < len(map)):
        pos = [(position[0] + 1), position[1]]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
    # To Up
    if((position[1] - 1) >= 0):
        pos = [position[0] ,(position[1] - 1)]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
    # To Down
    if((position[1] + 1) < len(map[0])):
        pos = [position[0] ,(position[1] + 1)]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
    # To Upper Left
    if((position[0]-1)>=0 and (position[1] - 1) >= 0):
        pos = [(position[0]-1) ,(position[1] - 1)]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
    # To Upper Right
    if((position[0]-1)>=0 and (position[1] + 1) < len(map[0])):
        pos = [(position[0]-1) ,(position[1] + 1)]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
     # To Bottom Right
    if((position[0]+1) < len(map) and (position[1] + 1) < len(map[0])):
        pos = [(position[0]+1) ,(position[1] + 1)]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])
     # To Bottom Left
    if((position[0]+1) < len(map) and (position[1] - 1) >= 0):
        pos = [(position[0]+1) ,(position[1] - 1)]
        if not pos in path and not pos in back_up:
            h_n = calculate_distance_by_Euclied(pos,target)
            g_n = compute_R_value_in_RGB(map,pos) + prev_g_n
            f_n = g_n + h_n
            state = [f_n,pos,g_n]
            heap.addElement(state)
            back_up.append(state[1])

    selected_state = heap.removeElement()
    position = selected_state[1]
    prev_g_n = selected_state[2]
    path.append(position)
    back_up.remove(position)
    
    print("Heap Size--->")
    print(heap.heapSize)
    print("-------------------------------------")
    
    if heap.heapSize > max_number_of_elements_in_stack:
        max_number_of_elements_in_stack = heap.heapSize
        
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

