from collections import deque
import heapq

# ---------- Graph representation ----------
# node -> list of (neighbor, edge_cost)
graph = {
    'A': [('B', 1), ('C', 4)],
    'B': [('D', 2), ('E', 5)],
    'C': [('F', 3)],
    'D': [('G', 1)],
    'E': [('G', 2)],
    'F': [('G', 4)],
    'G': []
}


def reconstruct_path(parent, goal):
    """Rebuild path from start to goal using the parent map."""
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path


# ---------- 1. BFS (Breadth-First Search) ----------
def bfs(graph, start, goal):
    visited = {start}
    parent = {start: None}
    queue = deque([start])
    order = []

    while queue:
        node = queue.popleft()
        order.append(node)
        if node == goal:
            return reconstruct_path(parent, goal), order

        for neighbor, _ in graph[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = node
                queue.append(neighbor)

    return None, order


# ---------- 2. DFS (Depth-First Search) ----------
def dfs(graph, start, goal):
    visited = set()
    parent = {start: None}
    stack = [start]
    order = []

    while stack:
        node = stack.pop()
        if node in visited:
            continue
        visited.add(node)
        order.append(node)
        if node == goal:
            return reconstruct_path(parent, goal), order

        for neighbor, _ in graph[node]:
            if neighbor not in visited:
                if neighbor not in parent:
                    parent[neighbor] = node
                stack.append(neighbor)

    return None, order


# ---------- 3. DLS (Depth-Limited Search) ----------
def dls(graph, start, goal, limit):
    # stack entries: (node, depth, path_so_far)
    stack = [(start, 0, [start])]
    order = []

    while stack:
        node, depth, path = stack.pop()
        order.append(node)

        if node == goal:
            return path, order

        if depth < limit:
            for neighbor, _ in graph[node]:
                if neighbor not in path:          # avoid cycles
                    stack.append((neighbor, depth + 1, path + [neighbor]))

    return None, order


# ---------- 4. IDDFS (Iterative Deepening DFS) ----------
def iddfs(graph, start, goal, max_depth):
    for limit in range(max_depth + 1):
        path, order = dls(graph, start, goal, limit)
        print(f"  Depth limit {limit}: explored {order}")
        if path:
            return path
    return None


# ---------- 5. UCS (Uniform Cost Search) ----------
def ucs(graph, start, goal):
    pq = [(0, start)]                # (cumulative_cost, node)
    best_cost = {start: 0}
    parent = {start: None}
    order = []

    while pq:
        cost, node = heapq.heappop(pq)

        if cost > best_cost.get(node, float('inf')):
            continue                 # stale entry

        order.append(node)
        if node == goal:
            return reconstruct_path(parent, goal), order, cost

        for neighbor, weight in graph[node]:
            new_cost = cost + weight
            if new_cost < best_cost.get(neighbor, float('inf')):
                best_cost[neighbor] = new_cost
                parent[neighbor] = node
                heapq.heappush(pq, (new_cost, neighbor))

    return None, order, None


# ---------- Driver ----------
if __name__ == "__main__":
    start, goal = 'A', 'G'

    print("=== BFS ===")
    path, order = bfs(graph, start, goal)
    print("Explored:", order, "\nPath:", path)

    print("\n=== DFS ===")
    path, order = dfs(graph, start, goal)
    print("Explored:", order, "\nPath:", path)

    print("\n=== DLS (limit=2) ===")
    path, order = dls(graph, start, goal, limit=2)
    print("Explored:", order, "\nPath:", path)

    print("\n=== IDDFS (max_depth=4) ===")
    path = iddfs(graph, start, goal, max_depth=4)
    print("Path:", path)

    print("\n=== UCS ===")
    path, order, cost = ucs(graph, start, goal)
    print("Explored:", order, f"\nPath: {path} (cost={cost})")