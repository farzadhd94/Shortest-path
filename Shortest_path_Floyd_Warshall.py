# import the built-in time module
import time

# Grab Currrent Time Before Running the Code
start = time.time()

# Recursive function to print the path of given node `u` from source node `v`
def printPath(path, v, u, route):
    if path[v][u] == v:
        return
    printPath(path, v, path[v][u], route)
    modif=path[v][u]+1
    route.append(modif)
 
 
# Function to print the shortest path with its cost between all pairs of nodes
def printSolution(path,cost, n):
    for v in range(n):
        for u in range(n):
            if u != v:
                route = [v+1]
                printPath(path, v, u, route)
                modif=u+1
                route.append(modif)
                print(f'The shortest path from {v+1} —> {u+1} is', route,'with the total cost of',cost[v][u])

 
# Function to run the Floyd–Warshall algorithm
#(This algorithm works when there is no negetive weight in the netwrok)
def floydWarshall(Matrix):
 
    # total number of nodes in the `Matrix`
    n = len(Matrix)
 
    # cost matrix stores the shortest cost and path matrix stores the shortest route
    # initially, cost would be the same as the weight of an edge and path would be empty
    cost = Matrix.copy()
    path = [[None for x in range(n)] for y in range(n)]
 
    # initialize cost and path
    for v in range(n):
        for u in range(n):
            if v == u:
                path[v][u] = 0
            elif cost[v][u] != float('inf'):
                path[v][u] = v
            else:
                path[v][u] = -1
 
    # run FloydWarshall
    for k in range(n):
        for v in range(n):
            for u in range(n):
                # If node `k` is on the shortest path from `v` to `u`,
                # then update the value of cost[v][u] and path[v][u]
                if cost[v][k] != float('inf') and cost[k][u] != float('inf') \
                        and (cost[v][k] + cost[k][u] < cost[v][u]):
                    cost[v][u] = cost[v][k] + cost[k][u]
                    path[v][u] = path[k][u]
 

 
    # Print the shortest path and cost between all pairs of nodes
    printSolution(path,cost, n)
 
if __name__ == '__main__':
 
    # define infinity
    I = float('inf')
 
    # the Sioux Fall Network matrix
    # when there is not direct flow between two nodes the adjacency is infinite (=I)
    Matrix = [
         [0,6,4,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I],
         [6,0,I,I,I,5,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I],
         [4,I,0,4,I,I,I,I,I,I,I,4,I,I,I,I,I,I,I,I,I,I,I,I],
         [I,I,4,0,2,I,I,I,I,I,6,I,I,I,I,I,I,I,I,I,I,I,I,I],
         [I,I,I,2,0,4,I,I,5,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I],
         [I,5,I,I,4,I,I,2,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I],
         [I,I,I,I,I,I,0,3,I,I,I,I,I,I,I,I,I,2,I,I,I,I,I,I],
         [I,I,I,I,I,2,3,0,10,I,I,I,I,I,I,5,I,I,I,I,I,I,I,I],
         [I,I,I,I,5,I,I,10,0,3,I,I,I,I,I,I,I,I,I,I,I,I,I,I],
         [I,I,I,I,I,I,I,I,3,0,5,I,I,I,6,4,8,I,I,I,I,I,I,I],
         [I,I,I,6,I,I,I,I,I,5,0,6,I,4,I,I,I,I,I,I,I,I,I,I],
         [I,I,4,I,I,I,I,I,I,I,6,0,3,I,I,I,I,I,I,I,I,I,I,I],
         [I,I,I,I,I,I,I,I,I,I,I,3,0,I,I,I,I,I,I,I,I,I,I,4],
         [I,I,I,I,I,I,I,I,I,I,4,I,I,0,5,I,I,I,I,I,I,I,4,I],
         [I,I,I,I,I,I,I,I,I,6,I,I,I,5,0,I,I,I,3,I,I,3,I,I],
         [I,I,I,I,I,I,I,5,I,4,I,I,I,I,I,0,2,3,I,I,I,I,I,I],
         [I,I,I,I,I,I,I,I,I,8,I,I,I,I,I,2,0,I,2,I,I,I,I,I],
         [I,I,I,I,I,I,2,I,I,I,I,I,I,I,I,3,I,0,I,4,I,I,I,I],
         [I,I,I,I,I,I,I,I,I,I,I,I,I,I,3,I,2,I,0,4,I,I,I,I],
         [I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,4,4,0,6,5,I,I],
         [I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,I,6,0,2,I,3],
         [I,I,I,I,I,I,I,I,I,I,I,I,I,I,3,I,I,I,I,5,2,0,4,I],
         [I,I,I,I,I,I,I,I,I,I,I,I,I,4,I,I,I,I,I,I,I,4,0,2],
         [I,I,I,I,I,I,I,I,I,I,I,I,4,I,I,I,I,I,I,I,3,I,2,0]
    ]

 
    # Run Floyd–Warshall algorithm
    floydWarshall(Matrix)
    
# Grab Currrent Time After Running the Code
end = time.time()
    
#Subtract Start Time from The End Time
total_time = end - start
print("the total computation time is "+ str(total_time))