'''
https://www.w3schools.com/dsa/dsa_algo_graphs_fordfulkerson.php
'''

class Graph:
    def __init__(self, size):
        self.adj_matrix = [[0] * size for _ in range(size)]
        self.size = size
        self.vertex_data = [''] * size

    def add_edge(self, u, v, c):
        self.adj_matrix[u][v] = c

    def add_vertex_data(self, vertex, data):
        if 0 <= vertex < self.size:
            self.vertex_data[vertex] = data

    def dfs(self, s, t, visited=None, path=None):
        if visited is None:
            visited = [False] * self.size
        if path is None:
            path = []

        visited[s] = True
        path.append(s)

        if s == t:
            return path

        for ind, val in enumerate(self.adj_matrix[s]):
            if not visited[ind] and val > 0:
                result_path = self.dfs(ind, t, visited, path.copy())
                if result_path:
                    return result_path

        return None

    def fordFulkerson(self, source, sink):
        max_flow = 0

        path = self.dfs(source, sink)
        while path:
            path_flow = float("Inf")
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                path_flow = min(path_flow, self.adj_matrix[u][v])

            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                self.adj_matrix[u][v] -= path_flow
                self.adj_matrix[v][u] += path_flow

            max_flow += path_flow

            path_names = [self.vertex_data[node] for node in path]
            print("Path:", " -> ".join(path_names), ", Flow:", path_flow)

            path = self.dfs(source, sink)

        return max_flow

g = Graph(6)
vertex_names = ['s', 'v1', 'v2', 'v3', 'v4', 't']
for i, name in enumerate(vertex_names):
    g.add_vertex_data(i, name)

g.add_edge(0, 1, 3)  # s  -> v1, cap: 3
g.add_edge(0, 2, 7)  # s  -> v2, cap: 7
g.add_edge(1, 3, 3)  # v1 -> v3, cap: 3
g.add_edge(1, 4, 4)  # v1 -> v4, cap: 4
g.add_edge(2, 1, 5)  # v2 -> v1, cap: 5
g.add_edge(2, 4, 3)  # v2 -> v4, cap: 3
g.add_edge(3, 4, 3)  # v3 -> v4, cap: 3
g.add_edge(3, 5, 2)  # v3 -> t,  cap: 2
g.add_edge(4, 5, 6)  # v4 -> t,  cap: 6

source = 0; sink = 5

print("The maximum possible flow is %d " % g.fordFulkerson(source, sink))

