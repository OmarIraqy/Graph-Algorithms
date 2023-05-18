class Graph():
    def _init_(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
    def addEdge(self,i,j,weight):
        self.graph[i][j]=weight
        print(self.graph)
        

#Test init
G=Graph()
G._init_(3)
G.addEdge(1,2,5)

