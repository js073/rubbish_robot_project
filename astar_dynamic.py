import heapq


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __lt__(self, other):
        return self.f < other.f or (self.f == other.f and self.g < other.g)


def heuristic(node, goal):
    return abs(node.position[0] - goal.position[0]) + abs(node.position[1] - goal.position[1])


def astar_dynamic(maze, start, end, update_callback):
    start_node = Node(None, start)
    end_node = Node(None, end)
    print(f"Start Node: {start_node.position}, End Node: {end_node.position}")

    open_list = []
    heapq.heappush(open_list, (0, start_node))
    closed_list = set()

    while open_list:
        current_node = heapq.heappop(open_list)[1]
        print(f"Current node: {current_node.position}, Open list size: {len(open_list)}")

        if current_node == end_node:
            path = reconstruct_path(current_node)
            print(f"Path found: {path}")
            return optimise_path(maze, path)

        closed_list.add(current_node)
        explore_and_update_neighbors(current_node, maze, open_list, closed_list, end_node)

        new_maze = update_callback()
        if new_maze != maze:
            maze = new_maze
            print("Grid updated")
            explore_and_update_neighbors(current_node, maze, open_list, closed_list, end_node)

        print(f"Open list contains: {[node[1].position for node in open_list]}")
        if not open_list:
            print("Open list is empty. No path found.")
            break

    return None


def explore_and_update_neighbors(current_node, maze, open_list, closed_list, end_node):
    print(f"Exploring neighbors for: {current_node.position}")
    for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

        if not (0 <= node_position[0] < len(maze) and 0 <= node_position[1] < len(maze[0])):
            continue
        if maze[node_position[0]][node_position[1]] != 0:
            continue
        print(f"Neighbor {node_position} is traversable")

        new_node = Node(current_node, node_position)
        if new_node in closed_list:
            continue

        new_node.g = current_node.g + 1
        new_node.h = heuristic(new_node, end_node)
        new_node.f = new_node.g + new_node.h

        if any(open_node for open_node in open_list if new_node == open_node[1] and new_node.g >= open_node[1].g):
            continue

        heapq.heappush(open_list, (new_node.f, new_node))
        print(f"Added to open list: {new_node.position}")


def reconstruct_path(end_node):
    path = []
    current = end_node
    while current is not None:
        path.append(current.position)
        print(f"Path step: {current.position}")  # Debug line
        current = current.parent
    return path[::-1]


def is_path_affected(path, maze):
    for position in path:
        x, y = position
        if maze[x][y] != 0:
            return True
    return False


def find_affected_segment(path, maze):
    for i, position in enumerate(path):
        x, y = position
        if maze[x][y] != 0:
            return i, None
    return None, None


def compute_path_from(maze, start_node, end_node, closed_list):
    open_list = []
    heapq.heappush(open_list, (start_node.g + heuristic(start_node, end_node), start_node))

    while open_list:
        current_node = heapq.heappop(open_list)[1]

        if current_node == end_node:
            return reconstruct_path(current_node)

        closed_list.add(current_node)

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if not (0 <= node_position[0] < len(maze) and 0 <= node_position[1] < len(maze[0])):
                continue
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)
            if new_node in closed_list:
                continue

            new_node.g = current_node.g + 1
            new_node.h = heuristic(new_node, end_node)
            new_node.f = new_node.g + new_node.h

            if any(open_node for open_node in open_list if new_node == open_node[1] and new_node.g >= open_node[1].g):
                continue

            heapq.heappush(open_list, (new_node.f, new_node))

    return None


def merge_paths(new_segment, unaffected_segment):
    if not new_segment or not unaffected_segment:
        return new_segment or unaffected_segment
    return new_segment + unaffected_segment[1:]


def line_of_sight(maze, start, end):
    x1, y1 = start
    x2, y2 = end

    if x1 == x2:  # Vertical line
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if maze[x1][y] != 0:  # Non-zero values are considered obstacles
                return False
        return True

    elif y1 == y2:  # Horizontal line
        for x in range(min(x1, x2), max(x1, x2) + 1):
            if maze[x][y1] != 0:  # Non-zero values are considered obstacles
                return False
        return True

    return False  # Diagonal line of sight is not allowed


def optimise_path(maze, path):
    if not path or len(path) <= 2:
        return path

    optimised_path = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = i + 1
        while j < len(path) and line_of_sight(maze, path[i], path[j]):
            j += 1

        # The last point that had a line of sight
        optimised_path.append(path[j - 1])
        i = j - 1

    if optimised_path[-1] != path[-1]:
        optimised_path.append(path[-1])  # Ensure the last point is always included

    return optimised_path


def is_diagonal_clear(maze, pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return all(
        maze[x][y] == 0 for x in range(min(x1, x2), max(x1, x2) + 1) for y in range(min(y1, y2), max(y1, y2) + 1))
