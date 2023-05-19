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
        for v in range(self.verticesCount):
            vertex=Vertex()
            vertex._init_(v)
            self.verticesList.append(vertex)
        self.verticesList[startIndex].modify(0,None)
        

    def Relax(self,minVertex,destination):
        if destination.distance>minVertex.distance+self.Weight(minVertex,destination):
            destination.modify(minVertex.distance+self.Weight(minVertex,destination),minVertex)
            if self.directed==False:
                destination.modify(minVertex.distance+self.Weight(minVertex,destination),minVertex)
            return True
 

        
    def shortestPath(self,startIndex):
        path=[]
        counter=self.verticesCount   
        self.initializeSource(startIndex)
        self.BUILD_MIN_HEAP()
        while self.verticesCount != 0:
            minVertex=self.ExtractMin()
            path.append(minVertex)
            for i in range(counter):
                if self.graph[minVertex.index][i] !=0:
                    self.Relax(minVertex,self.verticesList[i])
        
                        
                        
        # print("Shortest :")
        for i in range(len(path)):
            print(f"index --> {path[i].index}   distance {path[i].distance}")


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
        parent = [None] * self.verticesCount

        weights[0]=0
        mstSet=[False] * self.verticesCount

        #set as root
        parent[0]= -1

        for cout in range (self.verticesCount):
            #find edge with smallest weight
            u=self.minWeight(weights,mstSet)

            mstSet[u]=True

            for v in range(self.verticesCount):
                if self.graph[u][v] > 0 and mstSet[v]== False  and weights[v]>self.graph[u][v]:
                    weights[v]=self.graph[u][v]
                    parent[v]=u
        self.printMST(parent)

    def minWeight(self,weight,mstSet):

        min = 10000000

        for v in range(self.verticesCount):
            if weight[v] < min and mstSet[v] == False:
                min = weight[v]
                min_index = v

        return min_index

    def printMST(self, parent):
        print("Edge \tWeight")
        for i in range(1, self.verticesCount):
            print(parent[i], "-", i, "\t", self.graph[i][parent[i]])


#Test 
G=Graph()
G._init_(5,True)
G.addEdge(0,1,1)
G.addEdge(0,2,6)
G.addEdge(1,2,2)
G.addEdge(1,3,1)
G.addEdge(2,3,2)
G.addEdge(2,4,5)
G.addEdge(3,4,5)
G.shortestPath(0)

# if __name__ == '__main__':
#     g = Graph()
#     g._init_(5,False)
#     g.graph = [[0, 2, 0, 6, 0],
#                [2, 0, 3, 8, 5],
#                [0, 3, 0, 0, 7],
#                [6, 8, 0, 0, 9],
#                [0, 5, 7, 9, 0]]

#     g.primMST()
