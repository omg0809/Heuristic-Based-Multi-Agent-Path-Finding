import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
import heapq
import time

def greedy(mat,cur):
    global actions, nx, ny
    bnx = cur[0]
    bny = cur[1]
    bna = cur[2]
    bcst = mat[bnx,bny,bna]
    print(bcst)
    for idx in range(5):
        cx,cy,ca = tact(cur,idx)
        if(cx>0 and cx<nx and cy>0 and cy<ny):# and map[cx,cy]>0):
            print(cx,cy,ca,mat[cx,cy,ca])
            if(mat[cx,cy,ca]>=0 and mat[cx,cy,ca]<=bcst):
                bcst = mat[cx,cy,ca]
                bnx = cx
                bny = cy
                bna = ca
                print(cx,cy,ca,bcst)
    return [bnx,bny,bna]

def heuristic(val):
    if val<0 or val>26:
        return 0
    return 6*(8/val)**3

def greedy_dyn(mat,cur,all_pos):
    global actions, nx, ny, prior, prior_fac
    bnx = cur[0]
    bny = cur[1]
    bna = cur[2]
    distance = 0
    for j in all_pos:
        if(j[0]!=cur[0] or j[1]!=cur[1]):
            val = dis(cur,j)
            distance += heuristic(val)
    bcst = mat[bnx,bny,bna]**2+distance #mat**2
    # print(bnx,bny,bcst)
    # for idx,a in enumerate(actions):
    for idx in range(5):
        cx,cy,ca = tact(cur,idx)
        if(cx>0 and cx<50 and cy>0 and cy<50 and map[cx,cy]>0):
            distance = prior[idx]*prior_fac
            for j in all_pos:
                if(j[0]!=cur[0] or j[1]!=cur[1]):
                    val = dis([cx,cy],j)
                    distance += heuristic(val)
            # print(cx,cy,mat[cx,cy]+distance,bcst)
            if(mat[cx,cy,ca]>=0 and mat[cx,cy,ca]**2+distance<bcst):
                bcst = mat[cx,cy,bna]**2+distance
                bnx = cx
                bny = cy
                bna = ca
        # print(cx,cy,ca,mat[cx,cy,ca]**2+distance)
    return [bnx,bny,bna]

def check(cur,gol):
    for i in range(len(cur)):
        if(dis(cur[i],gol[i])>2):
            return False
    return True

def dis(a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def recomplie(cur_pos,costs):
    n_agents = len(cur_pos)
    len_left = []
    for agnt in range(n_agents):
        st = cur_pos[agnt]
        h = 0
        for agt in range(n_agents):
            if(agt != agnt):
                h += heuristic(dis(st,cur_pos[agt]))
        ct = costs[st[0],st[1],st[2],agnt]+h
        len_left.append(ct)
    priority = np.argsort(len_left)
    np.flip(priority,0)
    nxt_pos = cur_pos
    # print(priority)
    for agnt in priority:
        nxt_pos[agnt] = greedy_dyn(costs[:,:,:,agnt],cur_pos[agnt],cur_pos)
        cur_pos[agnt] = nxt_pos[agnt]
    return nxt_pos

def tact(parent_node,act):
    omg = act
    if act>2:
        omg = 2 - act
    omg*=10
    theta = parent_node[2]*10 + omg
    if theta>=360:
        theta -= 360
    if theta<0:
        theta += 360
    theta_rad = np.pi * theta / 180
    init_rad = np.pi * parent_node[2]*10 / 180
    omg = omg*np.pi/180
    if omg == 0:
        cx = parent_node[0] + np.cos(np.pi * theta / 180)
        cy = parent_node[1] + np.sin(np.pi * theta / 180)
    else:
        del_sin = np.sin(theta_rad) - np.sin(init_rad)
        del_cos = np.cos(theta_rad) - np.cos(init_rad)
        cx = parent_node[0] + (del_sin/omg)
        cy = parent_node[1] - (del_cos/omg)

    return round(cx),round(cy),theta//10

def ract(parent_node,act):
    omg = act
    if act>2:
        omg = 2 - act
    omg *=-10
    theta = parent_node[2]*10 + omg
    omg = omg*np.pi/180
    theta_rad = np.pi * theta / 180
    init_rad = np.pi * parent_node[2]*10 / 180
    if theta>=360:
        theta -= 360
    if theta<0:
        theta += 360
    if omg == 0:
        cx = parent_node[0] - np.cos(np.pi * theta / 180)
        cy = parent_node[1] - np.sin(np.pi * theta / 180)
    else:
        del_sin = np.sin(theta_rad) - np.sin(init_rad)
        del_cos = np.cos(theta_rad) - np.cos(init_rad)
        cx = parent_node[0] - (del_sin/omg)
        cy = parent_node[1] + (del_cos/omg)

    return round(cx),round(cy),theta//10

def nonhol(starts,goals):
    tnow = time.time()
    global map, actions, nx, ny
    n_agents = len(goals)
    costs = np.ones([nx,ny,36,n_agents])*-1
    new_map = 0*map
    for agnt in range(n_agents):
        new_map = 0*map
        gx = goals[agnt][0]
        gy = goals[agnt][1]
        # print(gx,gy)
        queue = []
        for ang in range(36):
            costs[gx,gy,ang,agnt]=0
            queue.append([goals[agnt][0],goals[agnt][1],ang])
        while(len(queue)>0):
            cur = queue.pop(0)
            for a in range(5):
                cx,cy,ca = ract(cur,a)
                if(cx>0 and cx<nx and cy>0 and cy<ny and map[cx,cy]>0):
                    if(costs[cx,cy,ca,agnt]<0 or costs[cx,cy,ca,agnt]>costs[cur[0],cur[1],cur[2],agnt]+1):
                        costs[cx,cy,ca,agnt] = costs[cur[0],cur[1],cur[2],agnt]+1
                        new_map[cx,cy]+=1
                        queue.append([cx,cy,ca])

    cur_pos = starts
    reached = False
    paths = []
    while not reached:
        for agnt,j in enumerate(cur_pos):
            new_map[j[0],j[1]] = 50*agnt
        for agnt in range(n_agents):
            costs[cur_pos[agnt][0],cur_pos[agnt][1],cur_pos[agnt][2],agnt] += 1
        cur_pos = recomplie(cur_pos,costs)
        len_left = []
        for agnt in range(n_agents):
            st = cur_pos[agnt]
            ct = costs[st[0],st[1],agnt]
            len_left.append(ct)

        reached = check(cur_pos,goals)
        paths.append(cur_pos)
    arr = np.array(paths)
    print(time.time()-tnow)
    print(np.shape(arr))
    plt.imshow(new_map,cmap="gray")
    plt.show()
    return arr




actions = [[1,0],[0,1],[-1,0],[0,-1],[1,1],[-1,-1],[1,-1],[-1,1]]
prior = [9,3,5,7,2,6,4,8]
prior_fac = 0.
map = cv2.imread("map_small.png",0)
nx,ny = map.shape
nx = 50
ny = 50
# nonhol([[10,10,0],[40,40,0]],[[40,40],[10,10]]) #length = 39, time = 6.77, length = 43, time = 5.33
# nonhol([[10,10,0],[40,40,0],[10,40,0]],[[40,40],[10,10],[40,10]]) #length = 45, time = 8.677629709243774, length = 47, time = 7.12
nonhol([[10,10,0],[40,40,0],[10,40,0],[40,10,0]],[[40,40],[10,10],[40,10],[10,40]]) #length = 46, time = 10.961822509765625, length = 48, time = 9.02