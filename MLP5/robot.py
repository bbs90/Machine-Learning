from collections import deque

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        
        self.heading = 0  # 0 = up, 1 = right, 2 = down, 3 = left
        self.maze_dim = maze_dim
        

        self.dirs = [100,1,-100,-1]
        
        

        self.phase = 1  # first phase. Exploring

       

        self.vertices = [0]  
        self.edges = dict()  
        self.current = 0     
        self.explored = []   
        self.notexplored = [0]  
        half_dim = maze_dim // 2 
        base_goal = (half_dim-1) * 100 + (half_dim-1)  
        self.goals = [base_goal, base_goal+100, base_goal+101, base_goal+1] 
        self.path = []  
        self.prev = []  

    # Converts the sensors info according to the heading. The input is a list with three value (a,b,c)
	# where a=left, b=front, c=right (relative to where the robot is heading). This function converts
	# this relative values into absolute values (North, East, South, West), since it is easier to work
	# with absolute directions. 
    def sensor_data(self, sensors):
        if self.heading == 0:
            return (sensors[1],sensors[2],0,sensors[0])
        elif self.heading == 1:
            return (sensors[0],sensors[1],sensors[2],0)
        elif self.heading == 2:
            return (0,sensors[0],sensors[1],sensors[2])
        else:  # heading == 3
            return (sensors[2],0,sensors[0],sensors[1])

    # adds an edge between two nodes
    def add_edge(self, u, v):
        if u in self.edges:  
            if v not in self.edges[u]:  
                self.edges[u].add(v)  
        else:  
            self.edges[u] = set([v]) 
			
        if v in self.edges:
            if u not in self.edges[v]:
                self.edges[v].add(u) 
        else:
            self.edges[v] = set([u])

    # a BFS to compute all distances from the starting vertex. Given the vertex "start", it will compute the
 	# shortest path to every other node in the graph.
    def bfs(self, start):
        visited = []   # list of visited nodes
        inqueue = []   # list of nodes already on the queue
        self.distances = dict()  # min distance to every node
        self.prev = dict()       # previous node to every node
        queue = deque([(start,0)])  # queue for the BFS
        while(len(queue)>0):            # while queue not empty
            (node,d) = queue.popleft()  # extract next element from the  queue
            if node in visited:         # if the node has been already visited extract next node
                continue
            self.distances[node] = d    # store distance in distances distances
            visited.append(node)        # add node to visited list
            if node in self.edges:                 # if there are outcoming nodes from node
                for neighbor in self.edges[node]:    # get all the neighbor not
                    if neighbor not in visited and neighbor not in inqueue:
                        queue.append((neighbor,d+1)) # queue the neighbor,
                        inqueue.append(neighbor)     # mark it as it is in the queue
                        self.prev[neighbor] = node   # store the previous node  

	# decode obtains the i,j coordinates from a given node
    def decode(self, n):
        i = n//100
        j = n%100
        return (i,j)
    
    # distance from n1 to n2
    def dist(self, n1, n2):
        (i1, j1) = self.decode(n1)  
        (i2, j2) = self.decode(n2)
        return abs(i1-i2)+abs(j1-j2)  

    # computes distance from node to closest goal
    def dist_to_goal(self, node):
            d1 = self.dist(node, self.goals[0])  
            d2 = self.dist(node, self.goals[1])  
            d3 = self.dist(node, self.goals[2])
            d4 = self.dist(node, self.goals[3])
            return min(d1, d2, d3, d4)          

    # computes the new movement based on the current path and position
	# The path was already computed, and the next node is in path[0]
	# So we want to move from current to path[0]
	# Since these two nodes are consecutive, they must be either in the same column or row
    def new_movement(self):
        new_state = self.path[0]
        self.path.remove(new_state)
        #print "current:",self.current
        #print "new_state:",new_state
        (i1,j1) = self.decode(self.current)
        #print "current:",i1,j1
        (i2,j2) = self.decode(new_state)
        #print "new_state:",i2,j2
        if i1==i2: 
			# same row
	        # We are moving on a row (east or west)
            if self.heading==1: # right
                h = 0
                d = j2 - j1
                new_heading = 1    
            elif self.heading == 3: # left
                h = 0
                d = j1 - j2
                new_heading = 3     
            elif self.heading == 0: # up
                if j2 > j1:         # we need to go east
                    h = 90         
                    d = j2 - j1
                    new_heading = 1  
                else:                # we need to go west
                    h = -90          
                    d = j1 - j2
                    new_heading = 3   
            else: # self.heading == 2 -- down
                if j2 > j1:    # we need to go east     
                    h = -90          
                    d = j2 - j1
                    new_heading = 1
                else:              # we need to go west   
                    h = 90            
                    d = j1 - j2
                    new_heading = 3
        elif j1==j2: # same column (north or south)
            if self.heading == 0:# north
                h = 0                
                d = i2 - i1
                new_heading = 0     
            elif self.heading == 2: # south
                h = 0                 
                d = i1 - i2           
                new_heading = 2      
            elif self.heading == 1: # east
                if i2 > i1:        # we need to go north 
                    h = -90     
                    d = i2 - i1
                    new_heading = 0    
                else:               # we need to go south  
                    h = 90      
                    d = i1 - i2
                    new_heading = 2   
            else: # self.heading == 3 -- left
                if i2 > i1:    # we need to go north     
                    h = 90         
                    d = i2 - i1
                    new_heading = 0   
                else:          # we need to go south         
                    h = -90          
                    d = i1 - i2
                    new_heading = 2    
        else:       # we should not get here
            print "Error"

        self.current = new_state
        self.heading = new_heading
        return (h,d)

    # next move for the exploring phase. look for the closer open space
	# There are two types of nodes. Explored nodes and notexplored nodes.
	# A notexplored node is a node that we know how to get to, but we have not
	# visited yet.
    def next_move_phase1(self, sensors):
        # if we are currently in a goal, we need to go to the second phase
        # because we now know how to get there
        if self.current in self.goals:  
		                                
            self.phase = 2
            
            # compute the path from start to end
            self.bfs(0)

            
            # build path to bestNode
            self.path = []                       
            while self.current != 0:            
                self.path.append(self.current)
                self.current = self.prev[self.current]

            self.path.reverse()

            self.current = 0  # reset the variables for second phase
            self.heading = 0  # going north

            return ('Reset', 'Reset')  
       
        # if we are here, we have not reached a goal yet
        # get the coordinates of the current position
        sensors_data = self.sensor_data(sensors)  
        #print "sensors_data:", sensors_data
        dirs = self.dirs  # list of directions we can go to
        if self.current in self.explored:   # if we are in an already explored node, it means there is a path we are
		                                    # following, so simply get the next direction and follow it
		                                    
            retval = self.new_movement()
            return retval
        else: # the current position has not been explored
            for direction in range(0,4): # we need to go to every direction we can up, right down and then left           
                if sensors_data[direction] > 0:   # if there is no wall  
                    for k in range(1,sensors_data[direction]+1):  
                        new_state = self.current + dirs[direction]*k  
                        if new_state not in self.explored and new_state not in self.notexplored: 
                            self.notexplored.append(new_state)   
                    for k in range(0, sensors_data[direction]):  
					                                                
                        for k2 in [1,2,3]:  # We can only go 1,2 or 3 steps                        
                            k3 = k + k2
                            if k3 > sensors_data[direction]:    # if we reached the end of the path then break 
                                break                              
                            state1 = self.current + dirs[direction]*k   #computing nodes
                            state2 = self.current + dirs[direction]*k3
                            self.add_edge(state1, state2)  # adding edges between nodes             
							
            
            self.notexplored.remove(self.current)  
            self.explored.append(self.current)     

			
            self.bfs(self.current)

            best_node = -1           # stores the best node (were we need to go to)
            best_dist = 9999999      # best distance (we need to minimize this)
            dist_to_goal = 9999999   # min distance to the goal (also minimize it) 
            for node,d in self.distances.iteritems(): # check every distance to get the minimum 
                if node in self.explored:  # node already explored then continue      
                    continue
                if d>0 and d < best_dist:   # d>0 (not the current node, and this is a best node)
                    best_node = node        # update the best node   
                    best_dist = d           # update the best distance
                    dist_to_goal = self.dist_to_goal(node) # compute the distance to the goal
                elif d>0 and d == best_dist: # there is a tie (check the distance to the goal)
                    new_dist = self.dist_to_goal(node)  # compute distance to goal
                    if new_dist < dist_to_goal:         # check if it is better
                        dist_to_goal = new_dist          # update all variables 
                        best_node = node
                        best_dist = d

            
			# build path to bestNode:	
            self.path = []
            while best_node != self.current:
                #print "best_node", best_node
                self.path.append(best_node)
                best_node = self.prev[best_node]

            self.path.reverse() # reverse the path since the previous iteration obtains the path reversed

            retval = self.new_movement() # get the next movement and return it
            return retval
            
        return (0,0)   #should not get here
        
    # in the second phase we already computed the path at the end of the
	# first phase, so we only need to get to next movement from the path list
    def next_move_phase2(self):

        if len(self.path)>0:
            return self.new_movement()
        
		

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

		# select which function to use, depending on the current phase
        if self.phase==1:
            return self.next_move_phase1(sensors)
        else:
            return self.next_move_phase2()
            
		# python tester.py test_maze_01.txt
		# python showmaze.py test_maze_01.txt

