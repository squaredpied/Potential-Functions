#!/usr/bin/python3
import numpy as np
import sys
from matplotlib import pyplot as plt
from PIL import Image
import math

# Function to find possible valid motions from a given position on the map
def possible_motions(map, position):
    # Get the dimensions of the map
    x = map.shape[1]
    y = map.shape[0]

    # Initialize an empty list to store valid neighboring positions
    possible = []

    # Check and append valid neighboring positions based on map boundaries and obstacle-free cells
    if position[1] - 1 >= 0 and map[position[0], position[1] - 1] == 0:
        possible.append((position[0], position[1] - 1))
    if position[0] - 1 >= 0 and map[position[0] - 1, position[1]] == 0:
        possible.append((position[0] - 1, position[1]))
    if position[1] + 1 < x and map[position[0], position[1] + 1] == 0:
        possible.append((position[0], position[1] + 1))
    if position[0] + 1 < y and map[position[0] + 1, position[1]] == 0:
        possible.append((position[0] + 1, position[1]))
    if position[1] - 1 >= 0 and position[0] - 1 >= 0 and map[position[0] - 1, position[1] - 1] == 0:
        possible.append((position[0] - 1, position[1] - 1))
    if position[1] + 1 < x and position[0] - 1 >= 0 and map[position[0] - 1, position[1] + 1] == 0:
        possible.append((position[0] - 1, position[1] + 1))
    if position[1] + 1 < x and position[0] + 1 < y and map[position[0] + 1, position[1] + 1] == 0:
        possible.append((position[0] + 1, position[1] + 1))
    if position[1] - 1 >= 0 and position[0] + 1 < y and map[position[0] + 1, position[1] - 1] == 0:
        possible.append((position[0] + 1, position[1] - 1))

    # Return the list of valid neighboring positions
    return possible

def eucl_dis(point1,point2):
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

# Wavefront Planner using Euclidean distance with Attraction Potential
def wavefront_planner_euclidean(map, goal):
    # Create an attraction potential map, initially identical to the given map
    attract_pot = map
    # Mark the goal cell in the attraction potential map with a value of 2
    attract_pot[goal[0], goal[1]] = 2
    # Initialize the queue with the goal position
    queue = [goal]

    # Continue the wavefront expansion until the queue is empty
    while len(queue) != 0:
        # Create a new queue for the next wavefront expansion
        new_queue = []

        # Iterate over each position in the current queue
        for position in queue:
            # Explore neighbors of the current position
            for neighbour in possible_motions(map, position):
                # Calculate the value for the current neighbor based on the Euclidean distance
                value = attract_pot[position[0], position[1]] + eucl_dis(neighbour, position)
                
                # Update the attraction potential map with the calculated value
                if attract_pot[neighbour[0], neighbour[1]] == 0:
                    # If the neighbor is unexplored, set its value to the calculated value
                    attract_pot[neighbour[0], neighbour[1]] = value
                else:
                    # If the neighbor has been visited before, update with the minimum value
                    attract_pot[neighbour[0], neighbour[1]] = min(value, attract_pot[neighbour[0], neighbour[1]])

                # Add the neighbor to the new queue for the next iteration
                new_queue.append(neighbour)

        # Update the current queue with the new queue
        queue = new_queue

    # Return the attraction potential map with updated values after wavefront expansion
    return attract_pot

# Function to perform Brushfire algorithm on a binary map
def brushfire(map):
    # Find the coordinates of obstacle cells (value 1) in the map
    x, y = np.where(map == 1)
    
    # Initialize a queue with obstacle coordinates
    queue = []
    for f, b in zip(x, y):
        queue.append((f, b))

    # Continue the Brushfire algorithm until the queue is empty
    while len(queue) != 0:
        # Create a new queue for the next iteration
        new_queue = []

        # Iterate over each position in the current queue
        for pos in queue:
            # Explore neighbors of the current position
            for neigh in possible_motions(map, pos):
                # Calculate the value for the current neighbor based on the Euclidean distance
                val = map[pos[0], pos[1]] + eucl_dis(neigh, pos)

                # Update the map with the calculated value if the neighbor is unexplored (map value is 0)
                if map[neigh[0], neigh[1]] == 0:
                    map[neigh[0], neigh[1]] = val
                else:
                    # If the neighbor has been visited before, update with the minimum value
                    map[neigh[0], neigh[1]] = min(map[neigh[0], neigh[1]], val)

                # Add the neighbor to the new queue for the next iteration
                new_queue.append(neigh)

        # Update the current queue with the new queue
        queue = new_queue

    # Return the map with updated values after Brushfire algorithm
    return map



def find_minimum(attract_function, pos):
    # Gets the neighbour of a cell with the minimum potential
    minimum=pos
    if pos[1]-1 >= 0 and attract_function[pos[0],pos[1]-1]!=1 and attract_function[pos[0],pos[1]-1]<attract_function[pos[0],pos[1]]:
        minimum=(pos[0],pos[1]-1)
    if pos[0]-1 >=0 and attract_function[pos[0]-1, pos[1]]!=1 and attract_function[pos[0]-1, pos[1]]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0]-1,pos[1])
    if pos[1]+1<attract_function.shape[1] and attract_function[pos[0],pos[1]+1]!=1 and attract_function[pos[0],pos[1]+1]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0],pos[1]+1)
    if pos[0]+1<attract_function.shape[0] and attract_function[pos[0]+1,pos[1]]!=1 and attract_function[pos[0]+1,pos[1]]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0]+1,pos[1])
    if pos[1]-1>=0 and pos[0]-1>=0 and attract_function[pos[0]-1,pos[1]-1]!=1 and attract_function[pos[0]-1,pos[1]-1]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0]-1, pos[1]-1)
    if pos[0]-1>=0 and pos[1]+1<attract_function.shape[1] and attract_function[pos[0]-1,pos[1]+1]!=1 and attract_function[pos[0]-1,pos[1]+1]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0]-1, pos[1]+1)
    if pos[1]+1<attract_function.shape[1] and pos[0]+1<attract_function.shape[0] and attract_function[pos[0]+1,pos[1]+1]!=1 and attract_function[pos[0]+1,pos[1]+1]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0]+1,pos[1]+1)
    if pos[1]-1>=0 and pos[0]+1<attract_function.shape[0] and attract_function[pos[0]+1,pos[1]-1]!=1 and attract_function[pos[0]+1,pos[1]-1]<attract_function[minimum[0],minimum[1]]:
        minimum=(pos[0]+1,pos[1]-1)
    return minimum

def find_the_path(potential_function, start):
    path=[start]
    posit=start
    isGoal=False
    while isGoal == False:
        minimum=find_minimum(potential_function,posit)
        if minimum==posit:
            isGoal=True
        else:
            path.append(minimum)
            posit=minimum
    return path

def repulsive_function(dis_obs, Q):
    dis_obs[dis_obs>Q]=0
    dis_obs[dis_obs==1]=1
    dis_obs[(dis_obs<=Q) & (dis_obs>1)]=4*((1/dis_obs[(dis_obs<=Q) & (dis_obs>1)])-(1/Q))**2
    return dis_obs

def potential_function(attraction, repulsion):
    attraction[attraction==1]= attraction.max()+1
    attraction=(attraction-attraction.min())/(attraction.max()-attraction.min())
    potential=attraction+repulsion
    # potential=(potential-potential.min())/(potential.max()-potential.min())    
    return potential

if __name__=="__main__":
    # Load grid map
    image = Image.open(sys.argv[1]).convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0],
    image.size[1])/255
    # binarize the image
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    # Invert colors to make 0 -> free and 1 -> occupied
    grid_map = (grid_map * -1) + 1
    # Show grid map
    start_point=(int(sys.argv[2]),int(sys.argv[3]))
    end_point=(int(sys.argv[4]),int(sys.argv[5]))


    X=wavefront_planner_euclidean(grid_map.copy(), end_point)
    path=find_the_path(X,start_point)
    print("Path using the attraction function = ", path)
    dis_to_obs=brushfire(grid_map.copy())
    repulse=repulsive_function(dis_to_obs.copy(),int(sys.argv[6]))

    potential=potential_function(X,repulse)
    sec_path=find_the_path(potential,start_point)
    print('\n\n')
    print("Path using potential function = ", sec_path)
    path2=potential.copy()

    #show the grid map
    plt.matshow(grid_map)
    plt.colorbar()

    #show the attraction function
    plt.matshow(X)
    plt.colorbar()

    path1=X.copy()

    #show the path to goal using the attraction function
    plt.matshow(path1)
    xax=[i for i,j in path]
    yax=[j for i,j in path]
    plt.plot(yax,xax, c='r')
    plt.scatter([start_point[1],end_point[1]],[start_point[0],end_point[0]],marker='+',c='r')
    plt.colorbar()

    #show brushfire result
    plt.matshow(dis_to_obs)
    plt.colorbar()

    #Plot the repulsive function
    plt.matshow(repulse)
    plt.colorbar()

    #plot the potential function
    plt.matshow(potential)
    plt.colorbar()

    #plot the path to goal using the potential function
    plt.matshow(path2)
    xax=[i for i,j in sec_path]
    yax=[j for i,j in sec_path]
    plt.plot(yax,xax, c='r')
    plt.scatter([start_point[1],end_point[1]],[start_point[0],end_point[0]],marker='+',c=['r','g'])
    plt.colorbar()
    plt.show()