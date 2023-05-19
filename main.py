import sys
import heapq

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
        predecessor = [-1] * self.verticesCount

        # Creating distance array to carry distance of every index from source
        distance = [sys.maxsize] * self.verticesCount

        #Setting distance from source to the source node to zero
        distance[src] = 0

        # Initilaizing a visited vertices array
        visited = [False] * self.verticesCount

        # Setting distance of source vertex as zero
        heap = [(0, src)]


        while heap:
            #Popping vertex with smallest distance
            (dist, u) = heapq.heappop(heap)

            #if already visited skip
            if visited[u]:
                continue

            #Set vertex as visited
            visited[u] = True

            #Update distance and predecessor of the Vertex
            for v in range(self.verticesCount):
                if self.graph[u][v] > 0 and not visited[v]:
                    new_distance = distance[u] + self.graph[u][v]
                    if new_distance < distance[v]:
                        distance[v] = new_distance
                        predecessor[v] = u
                        heapq.heappush(heap, (distance[v], v))

        #Printing the Distance from source to Each vertex and the predecessors of each vertex
        self.printSPT(distance,predecessor)



    def printSPT(self, dist,pred):
        for node in range(self.verticesCount):
            print("Vertix",node, "\t Distance from Source = ", dist[node],"\t Predecessor is  ",pred[node])

    def primMST(self):
        #Setting distance to reach each Vertex to infinity
        distance = [sys.maxsize]*self.verticesCount

        #Making array of predecessors all none for starters
        predecessor = [None] * self.verticesCount

        #Settting distance to reach first node as o
        distance[0]=0

        #Making array of visisted to check if a vertex has been visited or not
        visited=[False] * self.verticesCount

        #set first vertex as root (Deosn't have a predecessor)
        predecessor[0]= -1

        #Pushing (distance , first vertex) in the heap
        heap = [(0, 0)]

        while heap:
            #pop vertex with smallest distance
            (dist, u) = heapq.heappop(heap)

            #Check if its visited
            if visited[u]:
                continue

            #Set vertex as visited
            visited[u] = True

            #Update the vertex distance to reach the vertex and its predecessor
            for v in range(self.verticesCount):
                if self.graph[u][v] > 0 and not visited[v] and distance[v]>self.graph[u][v]:
                    distance[v] =  self.graph[u][v]
                    predecessor[v] = u
                    heapq.heappush(heap, (distance[v], v))

        #Functuon to print Edges and Weights
        self.printMST(predecessor)

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

