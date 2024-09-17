'''
https://www.geeksforgeeks.org/max-flow-problem-introduction/
'''
from collections import deque

# Number of vertices in the given graph
V = 6

# Returns True if there is a path from source 's' to sink 't' in
# residual graph. Also fills parent[] to store the path
def bfs(rGraph, s, t, parent):
    # Create a visited array and mark all vertices as not visited
    visited = [False] * V

    # Create a queue, enqueue source vertex and mark source vertex
    # as visited
    q = deque()
    q.append(s)
    visited[s] = True
    parent[s] = -1

    # Standard BFS Loop
    while q:
        u = q.popleft()
        for v in range(V):
            if visited[v] == False and rGraph[u][v] > 0:
                q.append(v)
                parent[v] = u
                visited[v] = True

    # If we reached sink in BFS starting from source, then return
    # True, else False
    return visited[t]

# Returns the maximum flow from s to t in the given graph
def fordFulkerson(graph, s, t):
    rGraph = [[0] * V for _ in range(V)]
    for u in range(V):
        for v in range(V):
            rGraph[u][v] = graph[u][v]

    parent = [-1] * V
    max_flow = 0

    # Augment the flow while there is a path from source to sink
    while bfs(rGraph, s, t, parent):
        path_flow = float('inf')
        v = t
        while v != s:
            u = parent[v]
            path_flow = min(path_flow, rGraph[u][v])
            v = parent[v]

        v = t
        while v != s:
            u = parent[v]
            rGraph[u][v] -= path_flow
            rGraph[v][u] += path_flow
            v = parent[v]

        max_flow += path_flow

    # Return the overall flow
    return max_flow

# Driver program to test above functions
if __name__ == "__main__":
    graph = [[0, 16, 13, 0, 0, 0],
             [0, 0, 10, 12, 0, 0],
             [0, 4, 0, 0, 14, 0],
             [0, 0, 9, 0, 0, 20],
             [0, 0, 0, 7, 0, 4],
             [0, 0, 0, 0, 0, 0]]

    print("The maximum possible flow is", fordFulkerson(graph, 0, 5))

