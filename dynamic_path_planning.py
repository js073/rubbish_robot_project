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


def heuristic(node, goal):
    return abs(node.position[0] - goal.position[0]) + abs(node.position[1] - goal.position[1])


def astar_dynamic(maze, start, end, update_callback):
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    heapq.heappush(open_list, (0, start_node))
    closed_list = set()

    path = []

    while open_list:
        current_node = heapq.heappop(open_list)[1]

        if current_node == end_node:
            path = reconstruct_path(current_node)
            return optimise_path(path, line_of_sight)

        closed_list.add(current_node)

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares
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

        new_maze = update_callback()
        if new_maze != maze:
            maze = new_maze
            if is_path_affected(path, maze):
                affected_segment_start, affected_segment_end = find_affected_segment(path, maze)
                if affected_segment_start is not None:
                    # Recompute the path from the start of the affected segment
                    new_path_segment = compute_path_from(maze, Node(None, path[affected_segment_start]), end_node,
                                                         closed_list)
                    if new_path_segment:
                        # Merge the new path segment with the unaffected part of the original path
                        path = merge_paths(new_path_segment, path[:affected_segment_start])
                        path = optimise_path(path, line_of_sight)  # Apply path smoothing
                    else:
                        return None  # No valid path found
            else:
                return path  # If the path is not affected, continue using the existing path

    return path  # Return the final path


def reconstruct_path(end_node):
    path = []
    current = end_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def is_path_affected(path, maze):
    for position in path:
        x, y = position
        if maze[x][y] != 0:  # Assuming 0 represents a traversable cell
            return True  # Path is affected if any position on the path is now blocked
    return False


def find_affected_segment(path, maze):
    affected_start = None
    affected_end = None
    for i, position in enumerate(path):
        x, y = position
        if maze[x][y] != 0:  # Assuming 0 represents a traversable cell
            if affected_start is None:
                affected_start = i
            affected_end = i
    return affected_start, affected_end


def compute_path_from(maze, start_node, end_node, closed_list):
    start_node.g = 0
    open_list = []
    heapq.heappush(open_list, (start_node.g + heuristic(start_node, end_node), start_node))

    while open_list:
        current_node = heapq.heappop(open_list)[1]

        if current_node == end_node:
            return reconstruct_path(current_node)

        closed_list.add(current_node)

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares
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

    common_node = new_segment[0]

    try:
        index_in_unaffected = unaffected_segment.index(common_node)
    except ValueError:
        return []

    return unaffected_segment[:index_in_unaffected] + new_segment


def line_of_sight(maze, start, end):
    """
    Check if there is a clear line of sight between two nodes in the grid.

    :param maze: The grid representing the environment.
    :param start: The starting node (x1, y1).
    :param end: The ending node (x2, y2).
    :return: True if there is a line of sight, False otherwise.
    """
    x1, y1 = start
    x2, y2 = end

    dx = x2 - x1
    dy = y2 - y1

    is_steep = abs(dy) > abs(dx)

    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    dx = x2 - x1
    dy = y2 - y1

    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        if maze[coord[0]][coord[1]] != 0:  # Assuming non-zero values are obstacles
            return False
        if coord not in points:
            points.append(coord)

        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    if swapped:
        points.reverse()

    return True


def optimise_path(maze, path):
    """
    Optimize the path by removing unnecessary intermediate nodes.

    :param maze: The grid representing the environment.
    :param path: The initial path generated by A*.
    :return: The optimized path.
    """
    optimized_path = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(maze, path[i], path[j]):
                optimized_path.append(path[j])
                i = j
                break
            j -= 1

        if j == i + 1:
            optimized_path.append(path[i + 1])
            i += 1

    return optimized_path

