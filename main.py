import sys
from heapq import heapify, heappush, heappop

class Graph():

    def init(self, vertices, directed):  # directed = True if it is directed graph, 0 otherwise
        self.verticesCount = vertices
        self.directed = directed
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
        self.verticesList = []

    def addEdge(self, i, j, weight):
        self.graph[i][j] = weight
        if self.directed == False:
            self.graph[j][i] = weight

    def dijkstra(self, src):

        # Creating predecssor array to carry predessor of every index
        pred = [None] * self.verticesCount
        # Creating distance array to carry distance of every index from source
        dist = [sys.maxsize] * self.verticesCount
        #Setting distance from source to the source node to zero
        dist[src] = 0
        # Initilaizing a visited vertices array
        visited = [False] * self.verticesCount

        for i in range(self.verticesCount):

            # x is always equal to src in first iteration
            x = self.minWeight(dist, visited)

            # Add the minimum distance vertex in the Shortest path tree
            visited[x] = True

            # Update dist value of the adjacent vertices and setting its predecessor
            for y in range(self.verticesCount):
                if self.graph[x][y] > 0 and visited[y] == False and dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = dist[x] + self.graph[x][y]
                    pred[y]=x

        self.printSPT(dist,pred)

    def printSPT(self, dist,pred):
        for node in range(self.verticesCount):
            print("Vertix",node, "\t Distance from Source = ", dist[node],"\t Predecessor is  ",pred[node])

    def primMST(self):
        weights = [100000]*self.verticesCount
        pred = [None] * self.verticesCount

        weights[0]=0
        visited=[False] * self.verticesCount

        #set as root
        pred[0]= -1

        for cout in range (self.verticesCount):
            #find edge with smallest weight
            u=self.minWeight(weights,visited)

            #Set Vertix as Visited
            visited[u]=True

            # Update the distance to reach adjecent Vertices and setting its predecessor
            for v in range(self.verticesCount):
                if self.graph[u][v] > 0 and visited[v]== False  and weights[v]>self.graph[u][v]:
                    weights[v]=self.graph[u][v]
                    pred[v]=u
        self.printMST(pred)

    def minWeight(self,weight,visited):

        min = 10000000

        for v in range(self.verticesCount):
            if weight[v] < min and visited[v] == False:
                min = weight[v]
                min_index = v

        return min_index

    def printMST(self, pred):
        print("Edge \tWeight")
        for i in range(1, self.verticesCount):
            print(pred[i], "-", i, "\t", self.graph[i][pred[i]])


#Test 
G=Graph()
G.init(5,False)
G.addEdge(0,3,1)
G.addEdge(0,1,6)
G.addEdge(1,4,2)
G.addEdge(1,2,5)
G.addEdge(1,3,2)
G.addEdge(2,4,5)
G.addEdge(3,4,1)
G.dijkstra(0)
G.primMST()

