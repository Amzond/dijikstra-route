import sys
import numpy as np



def main():
    
    g = Graph(9)
    g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], #0
            [4, 0, 8, 0, 0, 0, 0, 11, 0],   #1
            [0, 8, 0, 7, 0, 4, 0, 0, 2],    #2
            [0, 0, 7, 0, 9, 14, 0, 0, 0],   #3
            [0, 0, 0, 9, 0, 10, 0, 0, 0],   #4
            [0, 0, 4, 14, 10, 0, 2, 0, 0],  #5
            [0, 0, 0, 0, 0, 2, 0, 1, 6],    #6
            [8, 11, 0, 0, 0, 0, 1, 0, 7],   #7
            [0, 0, 2, 0, 0, 0, 6, 7, 0]]    #8
 
    starting_point = 0
    destinations = [7, 5, 6]
    
    # calculating distances to destinations from current node
    dist, pth = g.dijkstra(starting_point)
    
    # second node index
    index = closest_destination(destinations, dist)
    dist, path = g.dijkstra(index)
    curr = starting_point
    route = add_node_to_route(curr,index,path)

    print("closest node: ", index)
    print("route to closest node: ",route, "Distance: ", dist[starting_point])

def add_node_to_route(curr,index,path):
    route = []
    while curr != index:
        route.append(curr)
        curr = path[curr]
    route.append(index)
    
    return route


# finding closest destination 
def closest_destination(dest, dist):
    min_dist = sys.maxsize
    min_i = -1
    for d in dest:
        
        for de in dest:
            if (int(dist[int(de)]) < min_dist):
                min_i = int(de)
                min_dist = dist[int(de)]
                

        
   # print("min distance to node:", min_i)
    
    return int(min_i)
             

class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
 
    def printSolution(self, dist, path):
        print("Vertex \t Distance from Source \t Next node")
        for node in range(self.V):
            print(node, "\t", dist[node], "\t\t\t", path[node])
            

                
 
    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):
 
        # Initialize minimum distance for next node
        
        min = sys.maxsize
        min_index = 0
        # Search not nearest vertex not in the
        # shortest path tree
        for u in range(self.V):
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
 
        return min_index
 
    # Function that implements Dijkstra's single source
    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self,src):
 
        dist = [1e7] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
        path = [-1] * self.V
 
        for cout in range(self.V):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # x is always equal to src in first iteration
            x = self.minDistance(dist, sptSet)
     
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[x] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for y in range(self.V):
                if self.graph[x][y] > 0 and sptSet[y] == False and dist[y] > dist[x] + self.graph[x][y]:
                    path[y] = x
                    dist[y] = dist[x] + self.graph[x][y]

            
      #  self.printSolution(dist, path)        
        return dist, path
 
if __name__ == "__main__":
    main()