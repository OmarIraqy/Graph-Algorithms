from heapq import heapify, heappush, heappop
class Vertex():
    def _init_(self,index):
        self.distance=float('inf') 
        self.predecessor=None
        self.index=index

    def modify(self,distance,predecessor):
        self.distance=distance
        self.predecessor=predecessor
    
class Graph():
   
    def _init_(self, vertices,directed):#directed = True if it is directed graph, 0 otherwise
        self.verticesCount=vertices
        self.directed=directed
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
        self.verticesList=[]
    
    def addEdge(self,i,j,weight):
        self.graph[i][j]=weight
        if self.directed==False:
            self.graph[j][i]=weight
    
    def Weight(self,current,destination):
        return self.graph[current.index][destination.index]

    def initializeSource(self,startIndex):
        copy=[]
        for v in range(self.verticesCount):
            vertex=Vertex()
            vertex._init_(v)
            self.verticesList.append(vertex)
            copy.append(vertex)
        self.verticesList[startIndex].modify(0,None)
        copy[startIndex].modify(0,None)
        return copy
        

    def Relax(self,minVertex,destination):
        if destination.distance>minVertex.distance+self.Weight(minVertex,destination):
            
            destination.modify(minVertex.distance+self.Weight(minVertex,destination),minVertex)
            # print(f"new distance of {destination.index} is {destination.distance}")

 

        
    def shortestPath(self,startIndex,endIndex):
        counter=self.verticesCount   
        copy=self.initializeSource(startIndex)
        self.BUILD_MIN_HEAP()
        while self.verticesCount != 0:
            minVertex=self.ExtractMin()
            if minVertex.index==startIndex:
                startVertex=minVertex
            elif minVertex.index==endIndex:
                    endvertex=minVertex
            for i in range(counter):
                if self.graph[minVertex.index][i] !=0:
                    self.Relax(minVertex,copy[i])
        self.printPath(startVertex,endvertex)

    def printPath(self,startvertex,endvertex):
        if startvertex==endvertex:
            return
        current=endvertex
        stack=[]
        stack.append(endvertex)
        while current!= startvertex:
            # print(f"putting {current.index} of pre= {current.predecessor.index}")
            stack.append(current.predecessor)
            current=current.predecessor
        for i in range(len(stack)):
            vertex=stack.pop()
            print(f"to index -> {vertex.index} of distance {vertex.distance}")

                    


    def MIN_HEAPIFY(self, i):
        right = 2 * i + 2
        left = 2 * i + 1
        largest = i
        if left < self.verticesCount and self.verticesList[left].distance < self.verticesList[largest].distance:
            largest = left
        if right < self.verticesCount and self.verticesList[right].distance < self.verticesList[largest].distance:
            largest = right
        if largest != i:
            self.verticesList[i], self.verticesList[largest] = self.verticesList[largest], self.verticesList[i]
            self.MIN_HEAPIFY(largest)

    def BUILD_MIN_HEAP(self):
        for i in range(self.verticesCount // 2 - 1, -1, -1):
            self.MIN_HEAPIFY(i)
        

    def ExtractMin(self):
        min=self.verticesList[0]
        lastVertex= self.verticesList[self.verticesCount - 1]
        self.verticesList[0] = lastVertex
        self.verticesCount = self.verticesCount - 1
        self.MIN_HEAPIFY(0)
        return min

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
G._init_(5,False)

# G.addEdge(0,1,1)
# G.addEdge(0,2,6)
# G.addEdge(1,2,2)
# G.addEdge(1,3,1)
# G.addEdge(3,2,2)
# G.addEdge(2,4,5)
# G.addEdge(3,4,5)
# G.shortestPath(0,4)

G.addEdge(0,1,6)
G.addEdge(0,3,1)
G.addEdge(1,3,2)
G.addEdge(1,2,5)
G.addEdge(3,4,1)
G.addEdge(1,4,2)
G.addEdge(2,4,5)
G.shortestPath(0,2)

#     g = Graph()
#     g._init_(5,False)
#     g.graph = [[0, 2, 0, 6, 0],
#                [2, 0, 3, 8, 5],
#                [0, 3, 0, 0, 7],
#                [6, 8, 0, 0, 9],
#                [0, 5, 7, 9, 0]]

#     g.primMST()
