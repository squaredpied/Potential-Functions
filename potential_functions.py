#!/usr/bin/python3
import numpy as np
import sys
from matplotlib import pyplot as plt
from PIL import Image
import math

def possible_motions(map, position):
    x=map.shape[1]
    y=map.shape[0]

    possible=[]
    if position[1]-1 >= 0 and map[position[0],position[1]-1]==0:
        possible.append((position[0],position[1]-1))
    if position[0]-1 >=0 and map[position[0]-1, position[1]]==0:
        possible.append((position[0]-1,position[1]))
    if position[1]+1 <x and map[position[0],position[1]+1]==0:
        possible.append((position[0],position[1]+1))
    if position[0]+1<y and map[position[0]+1,position[1]]==0:
        possible.append((position[0]+1,position[1]))
    if position[1]-1>=0 and position[0]-1>=0 and map[position[0]-1,position[1]-1]==0:
        possible.append((position[0]-1,position[1]-1))
    if position[1]+1<x and position[0]-1>=0 and map[position[0]-1,position[1]+1]==0:
        possible.append((position[0]-1,position[1]+1))
    if position[1]+1<x and position[0]+1<y and map[position[0]+1,position[1]+1]==0:
        possible.append((position[0]+1,position[1]+1))
    if position[1]-1>=0 and position[0]+1<y and map[position[0]+1,position[1]-1]==0:
        possible.append((position[0]+1,position[1]-1))
    return possible

def eucl_dis(point1,point2):
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

def wavefront_planner_euclidean(map,goal):
    map[goal[0],goal[1]]=2
    queue=[goal]
    while len(queue) != 0:
        new_queue=[]
        for p in queue:
            for m in possible_motions(map,p):
                value=map[p[0],p[1]]+eucl_dis(m,p)
                if map[m[0],m[1]] == 0:
                    map[m[0],m[1]]=value
                else:
                    map[m[0],m[1]]=min(value,map[m[0],m[1]])
                new_queue.append(m)
        queue=new_queue
    return map

def check_neighbours(map, position):
    x=map.shape[1]
    y=map.shape[0]

    possible=[]
    if position[1]-1 >= 0 and (map[position[0],position[1]-1]==0):
        possible.append((position[0],position[1]-1))
    if position[0]-1 >=0 and (map[position[0]-1, position[1]]==0):
        possible.append((position[0]-1,position[1]))
    if position[1]+1 <x and (map[position[0],position[1]+1]==0):
        possible.append((position[0],position[1]+1))
    if position[0]+1<y and (map[position[0]+1,position[1]]==0):
        possible.append((position[0]+1,position[1]))
    if position[1]-1>=0 and position[0]-1>=0 and (map[position[0]-1,position[1]-1]==0):
        possible.append((position[0]-1,position[1]-1))
    if position[1]+1<x and position[0]-1>=0 and (map[position[0]-1,position[1]+1]==0):
        possible.append((position[0]-1,position[1]+1))
    if position[1]+1<x and position[0]+1<y and (map[position[0]+1,position[1]+1]==0):
        possible.append((position[0]+1,position[1]+1))
    if position[1]-1>=0 and position[0]+1<y and (map[position[0]+1,position[1]-1]==0):
        possible.append((position[0]+1,position[1]-1))
    return possible

def brushfire(map):
    x,y=np.where(map==1)
    queue=[]
    for f,b in zip(x,y):
        queue.append((f,b))
    #print(queue)
    while len(queue)!=0:
        new_queue=[]
        for p in queue:
            for neigh in check_neighbours(map, p):
                val=map[p[0],p[1]]+eucl_dis(neigh, p)
                if map[neigh[0],neigh[1]] ==0:
                    map[neigh[0],neigh[1]]=val
                else:
                    map[neigh[0],neigh[1]]=min(map[neigh[0],neigh[1]],val)
                new_queue.append(neigh)
        queue=new_queue
    return map



def find_minimum(attract_function, pos):
    minimum=pos
    if pos[1]-1 >= 0 and attract_function[pos[0],pos[1]-1]>1 and attract_function[pos[0],pos[1]-1]<attract_function[pos[0],pos[1]]:
        minimum=(pos[0],pos[1]-1)
    if pos[0]-1 >=0 and attract_function[pos[0]-1, pos[1]]>1 and attract_function[pos[0]-1, pos[1]]<attract_function[minimum[0],minimum[1]]:
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

def find_the_path(attract_function, start):
    path=[start]
    posit=start
    isGoal=False
    while isGoal == False:
        x=find_minimum(attract_function,posit)
        if x==posit:
            isGoal=True
        else:
            path.append(x)
            posit=x
        
    return path

def repulsive_function(dis_obs, Q):
    dis_obs[dis_obs>Q]=0
    dis_obs[dis_obs==1]=1
    dis_obs[(dis_obs<=Q) & (dis_obs>1)]=4*((1/dis_obs[(dis_obs<=Q) & (dis_obs>1)])-(1/Q))**2
    return dis_obs

def combining_function(attraction, repulsion):
    attraction[attraction==1]= attraction.max()+1
    attraction=(attraction-attraction.min())/(attraction.max()-attraction.min())
    potential=attraction+repulsion
    potential=(potential-potential.min())/(potential.max()-potential.min())    
    return potential

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

dis_to_obs=brushfire(grid_map.copy())
repulse=repulsive_function(dis_to_obs.copy(),int(sys.argv[6]))

potential=combining_function(X,repulse)

sec_path=find_the_path(potential,start_point)
path2=potential.copy()
for x,y in sec_path:
    path2[x,y]=1

plt.matshow(grid_map)
plt.colorbar()

plt.matshow(X)
plt.colorbar()

path1=X.copy()
for x,y in path:
    path1[x,y]=0.5
plt.matshow(path1)
plt.scatter([start_point[1],end_point[1]],[start_point[0],end_point[0]],marker='+',c='r')
plt.colorbar()

plt.matshow(dis_to_obs)
plt.colorbar()

plt.matshow(repulse)
plt.colorbar()

plt.matshow(potential)
plt.colorbar()

plt.matshow(path2)
plt.scatter([start_point[1],end_point[1]],[start_point[0],end_point[0]],marker='+',c=['r','g'])
plt.colorbar()
plt.show()