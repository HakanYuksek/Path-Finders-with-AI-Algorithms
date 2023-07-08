# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 17:46:59 2020

@author: USER
"""

# %% import Libraries
import numpy as np
import matplotlib.pyplot as plt
#from heap import HeapTree
from PIL import Image
import math
import time

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
            if k >= 0 and self.nodes[k] > self.nodes[i]:
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
                if self.nodes[right_child_index] < self.nodes[left_child_index]:
                    min_child = right_child_index
                else:
                    min_child = left_child_index
            elif left_child_index < self.heapSize:
                min_child = left_child_index
            else:
                min_child = i
            
            if self.nodes[i] > self.nodes[min_child]:
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

# This method calculates the distance between source and destination point
# By using Euclied Distance
def calculate_distance_by_Euclied(source, destination):
    dist = (source[0] - destination[0])**2 + (source[1]-destination[1])**2
    return dist


# %% Let's read the map from the file

map = np.array(Image.open("deneme4.png"))
plt.title("Harita (Resim)")
plt.imshow(map)
plt.show()

# %% Now let's begin the algorithm

# Firstly we need a start position
start = [0,0]

# Of course we also need a goal position
target = [(len(map)-1),(len(map[0])-1)]

# We will use a heap to hold states according to their point
heap = HeapTree()

# %% Actual algorithm part

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
        state = [point,pos]
        heap.addElement(state)
    # To Right
    if((position[0] + 1) < len(map)):
        pos = [(position[0] + 1), position[1]]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
    # To Up
    if((position[1] - 1) >= 0):
        pos = [position[0] ,(position[1] - 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
    # To Down
    if((position[1] + 1) < len(map[0])):
        pos = [position[0] ,(position[1] + 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
    # To Upper Left
    if((position[0]-1)>=0 and (position[1] - 1) >= 0):
        pos = [(position[0]-1) ,(position[1] - 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
    # To Upper Right
    if((position[0]-1)>=0 and (position[1] + 1) < len(map[0])):
        pos = [(position[0]-1) ,(position[1] + 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
     # To Bottom Right
    if((position[0]+1) < len(map) and (position[1] + 1) < len(map[0])):
        pos = [(position[0]+1) ,(position[1] + 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
     # To Bottom Left
    if((position[0]+1) < len(map) and (position[1] - 1) >= 0):
        pos = [(position[0]+1) ,(position[1] - 1)]
        # f_n =  (is distance to target by heuristic)
        point = calculate_distance_by_Euclied(pos, target)
        state = [point,pos]
        heap.addElement(state)
    selected_state = heap.removeElement()
    position = selected_state[1]
    
    path.append(position)

    print("Heap Size --->",heap.heapSize)
    if len(heap.nodes) > max_number_of_elements_in_stack:
        max_number_of_elements_in_stack = len(heap.nodes)
#%% We found the path, let's visualize it

end_time = time.time()

res_map = map.copy()

for each in path:
    res_map[each[0]][each[1]] = 1

plt.imshow(res_map)
plt.title("Siyah Çizgi Bulunan Path")
plt.show()


print("-------------------------------------")
print("Stack ten çekilen eleman sayisi--->",len(path))
print("-------------------------------------")
print("Stackte maksimum eleman sayisi--->",max_number_of_elements_in_stack)
print("-------------------------------------")
print("Geçen Süre--->"+str((end_time-start_time))+" saniye")

