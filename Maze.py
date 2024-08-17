import heapq
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import time

def dfs(maze, start, goal):
    stack = [(start, [start])]
    visited = set()
    while stack:
        (current, path) = stack.pop()
        if current in visited:
            continue
        visited.add(current)
        visualize_progress(maze, current, path, goal)
        for neighbor in get_neighbors(maze, current):
            if neighbor == goal:
                return path + [neighbor]
            stack.append((neighbor, path + [neighbor]))
    return None

def bfs(maze, start, goal):
    queue = deque([(start, [start])])
    visited = set()
    while queue:
        (current, path) = queue.popleft()
        if current in visited:
            continue
        visited.add(current)
        visualize_progress(maze, current, path, goal)
        for neighbor in get_neighbors(maze, current):
            if neighbor == goal:
                return path + [neighbor]
            queue.append((neighbor, path + [neighbor]))
    return None

def ucs(maze, start, goal):
    queue = [(0, start, [start])]
    visited = set()
    while queue:
        (cost, current, path) = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        visualize_progress(maze, current, path, goal)
        if current == goal:
            return path
        for neighbor in get_neighbors(maze, current):
            heapq.heappush(queue, (cost + 1, neighbor, path + [neighbor]))
    return None

def a_star(maze, start, goal):
    queue = [(0, start, [start])]
    visited = set()
    while queue:
        (cost, current, path) = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        visualize_progress(maze, current, path, goal)
        if current == goal:
            return path
        for neighbor in get_neighbors(maze, current):
            heuristic = manhattan_distance(neighbor, goal)
            heapq.heappush(queue, (cost + 1 + heuristic, neighbor, path + [neighbor]))
    return None

def best_first_search(maze, start, goal):
    queue = [(0, start, [start])]
    visited = set()
    while queue:
        (heuristic, current, path) = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        visualize_progress(maze, current, path, goal)
        if current == goal:
            return path
        for neighbor in get_neighbors(maze, current):
            heuristic = manhattan_distance(neighbor, goal)
            heapq.heappush(queue, (heuristic, neighbor, path + [neighbor]))
    return None

def get_neighbors(maze, position):
    rows, cols = len(maze), len(maze[0])
    row, col = position
    neighbors = []

    for r, c in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]:
        if 0 <= r < rows and 0 <= c < cols and maze[r][c] == 0:  # 0 represents a walkable cell
            neighbors.append((r, c))
    return neighbors

def manhattan_distance(position1, position2):
    return abs(position1[0] - position2[0]) + abs(position1[1] - position2[1])

def visualize_maze(maze, path=None):
    visual = [[' ' if cell == 0 else '1' for cell in row] for row in maze]
    if path:
        for r, c in path:
            visual[r][c] = '0'
        start, goal = path[0], path[-1]
        visual[start[0]][start[1]] = 'S'  # Start
        visual[goal[0]][goal[1]] = 'G'  # Goal
    for row in visual:
        print(''.join(row))
    print()

def visualize_progress(maze, current, path, goal):
    rows, cols = len(maze), len(maze[0])
    visual = np.zeros((rows, cols))
    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == 1:
                visual[r][c] = -1  # Mark walls as -1

    for r, c in path:
        visual[r][c] = 0.5  # Path as 0.5 (light gray)

    r, c = current
    visual[r][c] = 1  # Current position as 1 (white)

    gr, gc = goal
    visual[gr][gc] = 2  # Goal as 2 (green)

    plt.imshow(visual, cmap='gray', origin='upper')
    plt.show(block=False)
    plt.pause(0.1)
    plt.clf()

def read_maze():
    # Example of a predefined maze (0 = open path, 1 = wall)
    maze = [
        [0, 1, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0],
    ]
    return maze

def main():
    while True:
        print("Select a search algorithm:")
        print("1. Depth-First Search (DFS)")
        print("2. Breadth-First Search (BFS)")
        print("3. Uniform Cost Search (UCS)")
        print("4. A* Search")
        print("5. Best-First Search")
        
        choice = input("Enter your choice (1-5): ")
        
        algorithms = {
            '1': dfs,
            '2': bfs,
            '3': ucs,
            '4': a_star,
            '5': best_first_search
        }
        
        if choice not in algorithms:
            print("Invalid choice. Please try again.")
            continue
        
        maze = read_maze()
        start = (0, 0)  # Define the start position
        goal = (len(maze) - 1, len(maze[0]) - 1)  # Define the goal position
        
        algorithm = algorithms[choice]
        path = algorithm(maze, start, goal)
        
        if path:
            print("Path found:")
            visualize_maze(maze, path)
        else:
            print("No path found.")
        
        another = input("Solve another maze? (y/n): ")
        if another.lower() != 'y':
            break

if __name__ == "__main__":
    plt.ion()  # Turn on interactive mode for plotting
    main()
