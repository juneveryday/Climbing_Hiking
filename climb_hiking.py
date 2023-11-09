__author__ = "June Jin"
__version__ = "9.14.0 [Done]"

import heapq

class FloorGraph:

    def __init__(self, paths, keys):
        '''
        Function description:

        This function returns a tuple containing (the total time from start to exits, and route) where we need to visit.
        Between start and exits, we need to go take key for exit first.
        Before reach exits we need to take the key, it also takes time to get the key from the monster in the vertices.
        Therefore, This function will calculate the total time from start to take key, and reach to the exits.

        For the sum of minimum time for start to exits, it uses Dijkstra's algorithm and backtracking.

        Approach description:

        We need to make the extra vertices for key before start, linked with vertices which is original key location.
        I use extra vertices because we need to spend more time for taking key with defeating monster.
        Also, from the extra key vertices to original key vertices will be 0 because we make this path temporary vertices.
        After we take the key, we need to go back to the original key vertices. In here, I used multiverse, which is exactly same path with original map.

        From original map to multiverse, we need to add the total_vertices to each vertices with weight 0.
        And we climb again from the new vertices (temp key vertices) in multiuniverse to multiverse exit (exit vertices).

        If we record the whole path from start to exit, there will be some exactly same vertices (only number diff).

        I will check the key vertices and original vertices during backtracking.
        If the original graph vertices is same with the multiverse vertices, I will skip the multiverse vertices.

        Therefore, we can get the final route which is from start to exit with taking key in original map.

        :Input:
            argv1: paths : (u, v, x)
                u : non-negative integer, starting location ID for the path.
                v : non-negative integer, ending location ID for the path.
                x : non-negative integer, amount of the time need to spend duing path from u to v.

            argv2: keys : (k, y)
                k : non-negative integer, location ID of the key that can be found.
                y : non-negative integer, amount of the time need to spend to defeat the monster and retrieve the key.

        :Output, return:
            return (the total time from start to exits, and route) or None if there is no route.

        :Time complexity: O ( E + V ) where E is the number of edges and V is the number of vertices.
        :Aux space complexity: O ( E + V ) where E is the number of edges and V is the number of vertices.
        '''

        # max is finding the biggest number vertices from original input for find the number of vertices.
        max = 0

        for sample in paths:
            if sample[0] > max:
                max = sample[0]
            if sample[1] > max:
                max = sample[1]
                
        self.original_vertex = max

        # make more temp vertices for key.
        self.total_vertices = max+len(keys)+1 

        # preparing multiverse map.
        self.vertices = [None] * ((self.total_vertices) * 2) 

        # set every vertex information which is ((original_vertex + key) * 2) 
        # the reason multiply 2 is for multiverse map. O(V)
        for i in range(self.total_vertices):
            self.vertices[i] = Vertex(i)
            self.vertices[i+self.total_vertices] = Vertex(i+self.total_vertices)

        # set every vertex which is key location .key = True, O(V)
        for i in range(len(keys)):
            self.vertices[max+i+1].key = True
            self.vertices[max+i+1+self.total_vertices].key = True

        # make path between original key vertices and temp key vertices, O(V)
        # original key -> temp key ( weight will be the spending time for kill monster and get key.)
        # temp key -> original key ( weight will be 0 )
        for i in range(len(keys)): 
            u = keys[i][0]
            v = max+i+1
            w = keys[i][1]
            paths.append((u, v, w))
            paths.append((v, u, 0))

        # make path for multiverse map. O(E)
        new_paths = []
        for path in paths:
            new_paths.append((path[0]+self.total_vertices, path[1]+self.total_vertices, path[2]))

        # make path for original temp key vertices to multiverse key vertices. O(V)
        for i in range(len(keys)):
            v = max+i+1
            w = keys[i][1]
            paths.append((v, v+self.total_vertices, 0))

        # add edges paths and new paths , O(E)
        self.add_edges(paths)
        self.add_edges(new_paths)

    def climb(self, start, exits):
        '''
        Function description:

        A method that runs this function is djikstra's algorithm to find the shortest path from start to key and exit.
        It uses minheap from the start and discover the shortest next vertex with the weight.
        After discovering the all vertices, it will visit the shortest vertex ( it will be discovered ), continue till the weight from the start is all determined about all vertices.

        Approach description:

        There are 2 vertices that we need to find.
        First is key vertices and second is exit vertices. For the exit vertices, we must visit the key vertices first.

        Among whole exits, we will find the minimum distance from start to exit vertices.
        After we find the minimum distance, we will backtrack from the exit vertices to start vertices.
        (Special case : If there is no route, it will return None.)

        During backtracking, we will check the key vertices and original vertices.
        If the original vertices is same with the multiverse vertices, we will skip the multiverse vertices.

        :Input:
            argv1: start
                start : non-negative integer, starting vertex ID for the path in the floor.
                        it is only a single start.

            argv2: exits
                exits : non-empty list of non-negative integers, ending vertex IDs for the path in the floor.

        :Output, return:
            return (the total time from start to exits, and route) or None if there is no route.

        :Time complexity: O ( E log V ) where E is the number of edges and V is the number of vertices.
        :Aux space complexity: O ( E + V ) where E is the number of edges and V is the number of vertices.
        '''

        # make a new new exits for multiverse map.
        original_exits_length = len(exits)

        for i in range(len(exits)):
            exits.insert(len(exits)+i, exits[i]+self.total_vertices)

        # initialize the start vertex and its distance.
        key_source = self.vertices[start]
        key_source.distance = 0
    
        # make a minheap for all vertices.
        # push the start vertex and its distance.
        discovered = [] 
        heapq.heappush(discovered,(key_source,key_source.distance))
        key_source.discovered = True
    
        # it will loop until the minheap is empty.
        while len(discovered) > 0:
            u = heapq.heappop(discovered)
            u = u[0]
            u.visited = True

            # check every edges from u.
            for edge in u.edges: 
                v = self.vertices[edge.v]

                # if its never discovered, push it to the minheap.
                # add distance with the weight.
                if v.discovered == False and v.visited == False:
                    v.distance = u.distance + edge.w 
                    heapq.heappush(discovered,(v, v.distance))
                    v.discovered = True
                    v.previous = u

                # update the distance, if the new distance is smaller than the current distance.
                elif v.discovered == True and v.visited == False:
                    if v.distance > u.distance + edge.w:
                        v.distance = u.distance + edge.w
                        heapq.heappush(discovered,(v, v.distance))
                        v.previous = u
                        
        # find the minimum distance from the vertices which is exit.
        minimum_distance = float('inf')
        for i in exits[original_exits_length:]:
            if self.vertices[i].distance < minimum_distance:
                minimum_distance = self.vertices[i].distance
                final_exit = self.vertices[i]

        # if the minimum distance is infinity, it means there is no route.
        if final_exit.distance == 0:
            return None

        #make empty list.
        initial_route= [] 
        
        #backtracking from the exit vertices to start vertices.
        while final_exit is not key_source:
            initial_route.append(final_exit)
            if final_exit.previous is not None:
                final_exit = final_exit.previous
            else:
                break 

        initial_route.append(key_source)
        initial_route.reverse()

        output = []
        found_key = False

        for i in initial_route:

            if not found_key:

                # vertex which has the key.
                if i.key:
                    
                    multiverse_key = i.id
                    found_key = True

                else:
                    output.append(i.id)
            
            # after we found the key, skip the duplicated vertices in multiverse map.
            else:
                if i.id - self.total_vertices== multiverse_key:
                    continue
                
                elif i.id - self.total_vertices == output[-1]:
                    continue

                else:
                    output.append(i.id-self.total_vertices)
        
        return((minimum_distance,(output)))


    def add_edges(self, argv_edges):
        '''
        Function description:

        Function for adding edges in the graph.

        Approach description:

        It loops through the edges and add the edge in the current vertex.

        Input  : argv_edges : list of edges.
        :Output, return: None

        :Time complexity: O(E) where E is the number of edges.
        :Aux space complexity: O(1)
        '''
        for edge in argv_edges:
            u = edge[0]
            v = edge[1]
            w = edge[2]
            
            # make edges
            current_edge = Edge(u, v, w) 

            # edd edge in vertex u
            current_vertex = self.vertices[u] 
            current_vertex.add_edge(current_edge) 

class Vertex:

    def __init__(self, id) -> None:
        '''
        Function description:
        Initialize a new vertex with the given ID.
        My reference is from the lecutre and tutorial FIT2004.
        
        Approach description:
        it will initialize the vertex with the given ID with the following attributes.

        Attributes:
        id : id of vertex.
        edges : A list of the edges.
        previous : The previous vertex in the dijkstra's shortest path.
        distance : The distance between this vertex and the previous vertex in the dijkstra's shortest path.

        discovered : True if the vertex is found (pushed).
        visited : True if the vertex is visited (poped).

        :Time complexity: O(1)
        :Aux space complexity: O(1) 
        '''
        self.id = id

        # list
        self.edges = []

        # for traversal
        self.discovered = False
        self.visited = False

        # for distance
        self.distance = 0

        # backtracking
        self.previous = None

        # checking key
        self.key = False
        self.exit = False

    # magic method for comparing distance
    def __gt__(self, previous):
        return self.distance > previous.distance

    def __lt__(self, previous):
        return self.distance < previous.distance

    # add edge in the vertex
    def add_edge(self, edge):
        self.edges.append(edge)

class Edge:
    '''
    Edge that points from start source vertex u, to vertex v, with weight w.
    My reference is from the lecutre and tutorial FIT2004.
    
    Attributes:
    u : The vertex ID that the edge is pointing from.
    v : The vertex ID that the edge is pointing to.
    w : The weight of this edge.


    :Time complexity: O(1)
    :Aux space complexity: O(1)
    '''
    def __init__(self, u, v, w):
        self.u = u
        self.v = v
        self.w = w
