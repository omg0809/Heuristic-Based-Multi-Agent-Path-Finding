import numpy as np
import cv2
import time
from queue import PriorityQueue



class Node:
    def __init__(self, row, col, ori):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.ori = ori
        self.parent = None    # parent node
        self.cost = 0.0       # cost

class Environment():
    def __init__(self, map_path, no_robots, r = 0.5):
        img = np.zeros([500,500])
        self.map = cv2.imread(map_path,0)
        self.map_width, self.map_height = self.map.shape
        kernel = np.ones((10, 10), np.uint8)
        self.paths=[]
        self.step_theta=10
        self.parent=np.ones((self.map_width,self.map_height,360//self.step_theta),dtype=object)*-1
  
        # Using cv2.erode() method 
        self.padder = cv2.erode(self.map, kernel, cv2.BORDER_REFLECT) 
        self.cost = np.ones(self.parent.shape)*-1
        self.no_robots = no_robots
        self.width = r
        self.step_size=10

#---------------------------------------------------------------------------------------------------------------------------------------#
    
    def take_action(self,parent_node,act):
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
            cx = parent_node[0] + 10*np.cos(np.pi * theta / 180)
            cy = parent_node[1] + 10*np.sin(np.pi * theta / 180)
        else:
            del_sin = np.sin(theta_rad) - np.sin(init_rad)
            del_cos = np.cos(theta_rad) - np.cos(init_rad)
            cx = parent_node[0] + 10*(del_sin/omg)
            cy = parent_node[1] - 10*(del_cos/omg)

        return (int(round(cx)),int(round(cy)),theta//10)

    
#--------------------------------------------------------------------------------------------------------------------------------------------------#
    def check_max_len(self):
        max=0
        for i in self.paths:
            if len(i)>max:
                max=len(i)
        return max


#---------------------------------------------------------------------------------------------------------------------------------#    
    
    def reset(self,start,end):
        self.start = start
        self.end = end


#---------------------------------------------------------------------------------------------------------------------------------#    

    def if_valid(self,x,y,t):
        if (x<0 and y<0 and x>self.map_width and y>self.map_height):
            return False
        if(self.padder[x,y]==0):
            # print("Found obstacle")
            return False
        
        for i in range(len(self.paths)):
            t= int(t/10)
            if(t<len(self.paths[i]) and self.dis(self.paths[i][t],[x,y])<30):
                return False
        return True
            
#------------------------------------------------------------------------------------------------------------------------------------#    
    def get_child_cost(self,parent,child):
        return self.cost[parent[0],parent[1],parent[2]]+self.dis(parent,child)
#---------------------------------------------------------------------------------------------------------------------------------#    

    def dis(self, node1, node2):
        return np.sqrt((node1[0]-node2[0])**2+(node1[1]-node2[1])**2)

#---------------------------------------------------------------------------------------------------------------------------------#    
    def astar_non(self,start, goal):
        t_now=time.time()
        print("got star",start)
        print("got goal",goal)
        # print("begining paths lenght",len(self.paths),"path",self.paths)
        q=PriorityQueue()
        found = False
        q.put((0+self.dis(start,goal),start))
        vis = np.zeros((self.map_width, self.map_height,360//self.step_theta))
        
        self.parent[start[0]][start[1]][start[2]] =-1000 
        self.cost[start[0],start[1],start[2]] = 0
        count=0
        found=False
        while not q.empty():
            count+=1
            curr_node=q.get()[1]
            if(self.dis(curr_node,goal)<= 20 or curr_node==goal):
                end_node=curr_node
                found=True
                break
            for i in range(5):
                x,y,theta=self.take_action(curr_node,i)
                if self.if_valid(x,y,self.cost[curr_node[0]][curr_node[1]][curr_node[2]]+1) and self.parent[x][y][theta]==-1 and vis[x][y][theta]==0:
                    self.cost[x][y][theta]=self.get_child_cost(curr_node,[x,y,theta])
                    q.put((self.get_child_cost([x,y,theta],goal),[x,y,theta]))
                    self.parent[x][y][theta] =[curr_node[0],curr_node[1],curr_node[2]]

        if not found:
            print('No path found')
            return False
        last_node = end_node
        path=[]
        path.append(last_node)
        
        # Iterate until we reach the initial node
        while self.parent[last_node[0]][last_node[1]][last_node[2]] != self.parent[start[0],start[1],start[2]]:
            # Search for parent node in the list of closed nodes
            last_node =self.parent[last_node[0]][last_node[1]][last_node[2]]
            path.append(last_node)
        path.reverse()
        self.paths.append(path)
        print(self.paths[-1])
        print('time',t_now-time.time())

#---------------------------------------------------------------------------------------------------------------------------------#    



    def show_exploration(self, map_img):
        img_array=[]
        for j in self.paths:    
            for i in range(len(j) - 1, 0, -1):
                self.map[j[i][0],j[i][1]]=0 
        #         cv2.line(dummy, (j[i - 1][1],j[i - 1][0]),
        #                     (j[i][1], j[i][0]), (255,0,0))
        #     cv2.circle(dummy, (j[-1][1], j[-1][0]),
        #                 8, (0,0,255), -1)
        #     cv2.circle(dummy, (j[0][1], j[0][0]),
        #                 8,  (0,255,0), -1)
        
        # cv2.imshow("img",dummy)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
                
        
            
        # # # # Draw start and goal node to the video frame in the form of filled circle # # # # 
        

        max_len=self.check_max_len()
        for i in range(max_len):
            dummy=cv2.cvtColor(self.map,cv2.COLOR_GRAY2RGB)
            for j in self.paths:
                start=j[0]
                goal=j[-1]
                cv2.circle(dummy, (start[1],start[0]), 4, (0,255,0), -1)
                cv2.circle(dummy, (goal[1],goal[0]), 4, (0,0,255), -1)
                if len(j)>i:
                    cv2.circle(dummy, (j[i][1],j[i][0]), 4, (255,0,0), -1)
                else:
                    cv2.circle(dummy, (goal[1],goal[0]), 4, (255,0,0), -1)
            img_array.append(dummy)
        out = cv2.VideoWriter('new_video.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15,(500,500))
        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()

#---------------------------------------------------------------------------------------------------------------------------------#    


env = Environment('map.png',3)

# env.astar_non([50,50,0],[450,450,0])
env.astar_non([300,300,0],[100,20,0])
env.astar_non([50,50,0],[450,450,0])
# env.astar_non([400,400,0],[100,100,0])
# env.astar_non([100,400,0],[400,100,0])

# env.astar_non([400,400,0],[350,200,0])

# env.astar_non([250,50,0],[250,450,0])
# env.astar_non([300,450,0],[200,100,0])

# env.astar_non([350,50,0],[450,450,0])

# # env.astar_non([140,200,0],[300,420,0])
# env.astar_non([300,400,0],[100,80,0])
# env.astar_non([250,50,0],[250,450,0])


# env.astar_non([100,100,0],[400,400,0])
# env.astar_non([400,400,0],[100,100,0])
# # env.astar_non([100,400,0],[400,100,0])
# env.astar_non([27,40,0],[10,10,0])



# Vishrut test case

# env.astar_non([100,100,0],[400,400,0])
# env.astar_non([410,410,0],[110,110,0])
# env.astar_non([100,400,0],[400,100,0])
# env.astar_non([410,110,0],[110,410,0])

env.show_exploration('blank_map.jpg')
