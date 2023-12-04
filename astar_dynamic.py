import heapq
import rospy


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


def astar_dynamic(maze, start, end):
    rospy.loginfo("Starting A* with start: {} and end: {}".format(start, end))
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    heapq.heappush(open_list, (0, start_node))
    closed_list = set()

    while open_list:
        current_node = heapq.heappop(open_list)[1]


        if current_node == end_node:
            return reconstruct_path(current_node)

        closed_list.add(current_node)
        explore_and_update_neighbors(current_node, maze, open_list, closed_list, end_node)
    rospy.logwarn("No path found from {} to {}".format(start, end))
    return None  # Return None if no path is found



def explore_and_update_neighbors(current_node, maze, open_list, closed_list, end_node):

    for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        node_position = (int(current_node.position[0] + new_position[0]), int(current_node.position[1] + new_position[1]))

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
        


def reconstruct_path(end_node):
    path = []
    current = end_node
    while current is not None:
        path.append(current.position)
        print(f"Path step: {current.position}")  # Debug line
        current = current.parent
    return path[::-1]


def is_path_affected(path, maze):
    for step in path:
        x, y = int(step[0]), int(step[1])
        if maze[x][y] != 0:
            return True
    return False



def find_affected_segment(path, maze):
    affected_segment_start = None
    affected_segment_end = None
    
    for i, position in enumerate(path):
        x, y = position
        if maze[x][y] != 0:
            if affected_segment_start is None:
                # Start of a new affected segment
                affected_segment_start = position
            # Update the end to the current position as we don't know where the segment ends yet
            affected_segment_end = position
        else:
            if affected_segment_start is not None:
                # Found the end of the affected segment
                break
    
    return affected_segment_start, affected_segment_end




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


