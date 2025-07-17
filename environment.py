import os
from typing import List, Tuple
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq

import yaml

# ================================ basic objects ============================================

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    def __str__(self) -> str:
        return "({},{})".format(self.x, self.y)
    def __eq__(self, other) -> bool:
        if isinstance(other, Point):
            return self.x==other.x and self.y==other.y
        return False
    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self.x-other.x, self.y-other.y)
        else:
            raise Exception
    def __add__(self, other):
        if isinstance(other, Point):
            return Point(self.x+other.x, self.y+other.y)
        else:
            raise Exception
    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Point(self.x*other, self.y*other)
    def __rmul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Point(self.x*other, self.y*other)

class Line:
    def __init__(self, start, end):
        self.point1: Point = start
        self.point2: Point = end
        self.index: int = 0
    def __str__(self) -> str:
        return "{} - {}".format(self.point1, self.point2)
    def point(self, vertex: float = 0.5):
        if vertex < 0.0 or vertex > 1.0:
            raise Exception
        else:
            return self.point1+(self.point2-self.point1)*vertex

class Triangle:
    def __init__(self, v1, v2, v3):
        self.vertices = [v1, v2, v3]

# =========================== maklink graph with dijkstra ===================================

class MaklinkGraph:
    def __init__(self, V: List[Line]):
        self.size = len(V)
        self.maklink_lines: List[Line] = V
        self.adjacency_matrix = np.zeros((self.size+2, self.size+2), dtype=bool)
        for i in range(len(self.maklink_lines)):
            self.maklink_lines[i].index = i
        self.start_point: Point = None
        self.end_point: Point = None
        self.path: List[int] = None
        self.maklink_distances = None
        self.path_length = 0.0

    def add_edge(self, i, j):
        self.adjacency_matrix[i, j] = True
        self.adjacency_matrix[j, i] = True

    def add_start_point(self, start_point: Point, adjacent_edges: List[int]):
        self.start_point = start_point
        for a in adjacent_edges:
            self.adjacency_matrix[self.size, a] = True
            self.adjacency_matrix[a, self.size] = True

    def add_end_point(self, end_point: Point, adjacent_edges: List[int]):
        self.end_point = end_point
        for a in adjacent_edges:
            self.adjacency_matrix[self.size+1, a] = True
            self.adjacency_matrix[a, self.size+1] = True

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def calculate_numerical_adjacency_matrix(self):
        numerical_adjacency = np.zeros((self.size+2, self.size+2), dtype=float)
        for i in range(self.size):
            if self.adjacency_matrix[self.size][i]:
                numerical_adjacency[self.size][i] = numerical_adjacency[i][self.size] = self.calculate_distance(self.start_point, self.maklink_lines[i].point())
            else:
                numerical_adjacency[self.size][i] = numerical_adjacency[i][self.size] = math.inf
            if self.adjacency_matrix[self.size+1][i]:
                numerical_adjacency[self.size+1][i] = numerical_adjacency[i][self.size+1] = self.calculate_distance(self.end_point, self.maklink_lines[i].point())
            else:
                numerical_adjacency[self.size+1][i] = numerical_adjacency[i][self.size+1] = math.inf
            for j in range(i + 1, self.size):
                if self.adjacency_matrix[j][i]:
                    distance = self.calculate_distance(self.maklink_lines[i].point(), self.maklink_lines[j].point())
                    numerical_adjacency[i][j] = numerical_adjacency[j][i] = distance
                else:
                    numerical_adjacency[i][j] = numerical_adjacency[j][i] = math.inf
        return numerical_adjacency

    def dijkstra(self):
        adjacency = self.calculate_numerical_adjacency_matrix()

        distance = [float('inf')] * (self.size + 2)
        distance[self.size] = 0
        previous = [-1] * (self.size + 2)  # To store the previous node in the shortest path
        pq = [(0, self.size)]

        while pq:
            dist, current = heapq.heappop(pq)

            for neighbor in range(self.size + 2):
                if adjacency[current][neighbor] and distance[current] + adjacency[current][neighbor] < distance[neighbor]:
                    distance[neighbor] = distance[current] + adjacency[current][neighbor]
                    previous[neighbor] = current
                    heapq.heappush(pq, (distance[neighbor], neighbor))

        # Reconstruct the path from the end to the start
        path = []
        current = self.size + 1
        while current != -1:
            path.insert(0, current)
            current = previous[current]
        #print("Path: ", path)
        #print("Distances: ", distance)
        self.path = path
        self.maklink_distances = distance
        self.path_length = distance[-1]

# =============================== environment ===============================================

class Environment:
    def __init__(self, x, y):
        self.x_measure: float = x
        self.y_measure: float = y
        self.obstacles: List[Triangle] = []
        self.maklink: MaklinkGraph = None

    # ------------------------ add obstacle to environment ----------------------------

    def add_obstacle(self, vertices):
        if len(vertices) == 3 and all(self.is_within_bounds(v) for v in vertices):
            self.obstacles.append(Triangle(*vertices))
        else:
            print("Rejected obstacle. Not within environment bounds.")

    # ------------------------------ add waypoints ------------------------------------

    def add_point(self, point: Point):
        def check_line(line: Line, adjacent_lines: List[Line]):
            start_line = Line(point, line.point())
            for obst in self.obstacles:
                for i in range(3):
                    if self.do_lines_intersect(start_line, Line(obst.vertices[i], obst.vertices[(i + 1) % 3])):
                        return adjacent_lines
            for other_maklink in self.maklink.maklink_lines:
                if not other_maklink == line:
                    if self.do_lines_intersect(start_line, other_maklink):
                        return adjacent_lines
            adjacent_lines.append(line)
            return adjacent_lines
        adjacent_lines = []
        for line in self.maklink.maklink_lines:
            adjacent_lines=check_line(line, adjacent_lines)
        return [a.index for a in adjacent_lines]

    def add_start_point(self, point: Point):
        adjacent_lines = self.add_point(point)        
        self.maklink.add_start_point(point, adjacent_lines)

    def add_end_point(self, point: Point):
        adjacent_lines = self.add_point(point)        
        self.maklink.add_end_point(point, adjacent_lines)

    # ------------------------------ maklink graph construction -------------------------------

    def calculate_maklink(self):
        if self.maklink is not None:
            del self.maklink
        free_maklink_lines = self.find_free_maklink_lines()
        # Remove crossings
        self.remove_crossings(free_maklink_lines)
        # Create the Maklink graph
        self.create_maklink_graph(free_maklink_lines)

    def find_free_maklink_lines(self):
        free_maklink_lines = []
        # 2) Add straight lines connecting vertices of different obstacles
        for i, obstacle1 in enumerate(self.obstacles):
            for j, obstacle2 in enumerate(self.obstacles):
                if i != j:
                    for vertex1 in obstacle1.vertices:
                        for vertex2 in obstacle2.vertices:
                            line = Line(vertex1, vertex2)
                            if not self.does_line_intersect_obstacle(line) and not (vertex1 == vertex2) and not (self.is_on_bounds(vertex1) and self.is_on_bounds(vertex2)):
                            #if not self.does_line_intersect_obstacle(line) and not (vertex1 == vertex2):
                                free_maklink_lines.append(line)
        # 1) Add lines perpendicular to the environment boundary from obstacle vertices
        for obstacle in self.obstacles:
            for vertex in obstacle.vertices:
                if not self.is_on_bounds(vertex):
                    line1 = Line(vertex, Point(vertex.x, 0))
                    line2 = Line(vertex, Point(vertex.x, self.y_measure))
                    line3 = Line(vertex, Point(0, vertex.y))
                    line4 = Line(vertex, Point(self.x_measure, vertex.y))

                    if not self.does_line_intersect_obstacle(line1):
                        free_maklink_lines.append(line1)
                    if not self.does_line_intersect_obstacle(line2):
                        free_maklink_lines.append(line2)
                    if not self.does_line_intersect_obstacle(line3):
                        free_maklink_lines.append(line3)
                    if not self.does_line_intersect_obstacle(line4):
                        free_maklink_lines.append(line4)
        return free_maklink_lines

    def does_line_intersect_obstacle(self, line: Line):
        for obstacle in self.obstacles:
            for i in range(3):
                if self.do_lines_intersect(line, Line(obstacle.vertices[i], obstacle.vertices[(i + 1) % 3])):
                    return True
        return False

    def remove_crossings(self, lines: List[Line]):
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                if self.do_lines_intersect(lines[i], lines[j]):
                    lines.pop(j)
                    return self.remove_crossings(lines)

    def remove_intraadjacent_lines(self, lines: List[Line]):
        # TODO: implement to remove unnecessary maklink lines before constructing the graph
        pass

    def create_maklink_graph(self, lines: List[Line]):
        def angle_from_triangle(line: Line, intersect_point, triangle_point1, triangle_point2):
            # Determine which endpoint of the line is connected to intersect_point
            if line.point1 == intersect_point:
                line_endpoint = line.point2
            elif line.point2 == intersect_point:
                line_endpoint = line.point1
            else:
                raise ValueError("The line must have one endpoint equal to intersect_point.")

            # Calculate vectors for the line and triangle sides
            line_vector = Point(line_endpoint.x - intersect_point.x, line_endpoint.y - intersect_point.y)
            triangle_vector1 = Point(triangle_point1.x - intersect_point.x, triangle_point1.y - intersect_point.y)
            triangle_vector2 = Point(triangle_point2.x - intersect_point.x, triangle_point2.y - intersect_point.y)

            # Calculate angles using the cross product to determine counterclockwise direction
            angle1 = math.atan2(triangle_vector1.y, triangle_vector1.x) - math.atan2(line_vector.y, line_vector.x)
            angle2 = math.atan2(triangle_vector2.y, triangle_vector2.x) - math.atan2(line_vector.y, line_vector.x)

            # Ensure the angle is between 0 and 360 degrees
            angle1 = (angle1 + 2 * math.pi) % (2 * math.pi)
            angle2 = (angle2 + 2 * math.pi) % (2 * math.pi)

            # Return the minimum angle in degrees
            return math.degrees(min(angle1, angle2))

        def order_lines(lines: List[Line], triangle_point: Point, triangle_point1: Point, triangle_point2: Point) -> List[Line]:
            # Calculate angles for each line and store them in a dictionary
            angles = {}
            for line in lines:
                angles[line] = angle_from_triangle(line, triangle_point, triangle_point1, triangle_point2)
            # Sort lines based on angles in clockwise order
            sorted_lines = sorted(angles.keys(), key=lambda x: angles[x])
            return sorted_lines

        def wall_line(line1: Line, line2: Line, x_or_not: bool, zero_or_not: bool):
            if x_or_not:
                if zero_or_not:
                    if line1.point1.x == 0.0:
                        point_one = line1.point1
                    elif line1.point2.x == 0.0:
                        point_one = line1.point2
                    else: raise Exception
                    if line2.point1.x == 0.0:
                        point_two = line2.point1
                    elif line2.point2.x == 0.0:
                        point_two = line2.point2
                    else: raise Exception
                else:
                    if line1.point1.x == self.x_measure:
                        point_one = line1.point1
                    elif line1.point2.x == self.x_measure:
                        point_one = line1.point2
                    else: raise Exception
                    if line2.point1.x == self.x_measure:
                        point_two = line2.point1
                    elif line2.point2.x == self.x_measure:
                        point_two = line2.point2
                    else: raise Exception
            else:
                if zero_or_not:
                    if line1.point1.y == 0.0:
                        point_one = line1.point1
                    elif line1.point2.y == 0.0:
                        point_one = line1.point2
                    else: raise Exception
                    if line2.point1.y == 0.0:
                        point_two = line2.point1
                    elif line2.point2.y == 0.0:
                        point_two = line2.point2
                    else: raise Exception
                else:
                    if line1.point1.y == self.y_measure:
                        point_one = line1.point1
                    elif line1.point2.y == self.y_measure:
                        point_one = line1.point2
                    else: raise Exception
                    if line2.point1.y == self.y_measure:
                        point_two = line2.point1
                    elif line2.point2.y == self.y_measure:
                        point_two = line2.point2
                    else: raise Exception
            return Line(point_one, point_two)

        def share_common_point(line1: Line, line2: Line):
            return line1.point1 == line2.point1 or line1.point1 == line2.point2 or line1.point2 == line2.point1 or line1.point2 == line2.point2

        self.maklink = MaklinkGraph(lines)
        
        # connect lines sharing the same triangle vertex
        for obstacle in self.obstacles:
            for vertex in range(3):
                connected_lines = []
                for line in self.maklink.maklink_lines:
                    if line.point1 == obstacle.vertices[vertex] or line.point2 == obstacle.vertices[vertex]:
                        connected_lines.append(line)

                ordered_connected_lines = order_lines(connected_lines, obstacle.vertices[vertex], obstacle.vertices[(vertex + 1) % 3], obstacle.vertices[(vertex + 2) % 3])
                
                for i in range(len(connected_lines)-1):
                    self.maklink.add_edge(ordered_connected_lines[i].index, ordered_connected_lines[i+1].index)

        # connect lines connected to environment edge
        connected_lines1:List[Line] = [line for line in self.maklink.maklink_lines if line.point1.x == 0.0 or line.point2.x == 0.0]
        connected_lines2:List[Line] = [line for line in self.maklink.maklink_lines if line.point1.x == self.x_measure or line.point2.x == self.x_measure]
        connected_lines3:List[Line] = [line for line in self.maklink.maklink_lines if line.point1.y == 0.0 or line.point2.y == 0.0]
        connected_lines4:List[Line] = [line for line in self.maklink.maklink_lines if line.point1.y == self.y_measure or line.point2.y == self.y_measure]
        conlines: List[Tuple[List[Line],bool,bool]] = [(connected_lines1,True,True), (connected_lines2,True,False), (connected_lines3,False,True), (connected_lines4,False,False)]
        for connected_lines,param1,param2 in conlines:
            length = len(connected_lines)
            for i in range(length):
                for j in range(i,length):
                    wallline = wall_line(connected_lines[i], connected_lines[j],param1,param2)
                    if not self.does_line_intersect_obstacle(wallline) and not share_common_point(connected_lines[i], connected_lines[j]) and all((connected_lines[i] == maklink_line or connected_lines[j] == maklink_line) or not self.do_lines_intersect(Line(connected_lines[i].point(),connected_lines[j].point()), maklink_line) for maklink_line in self.maklink.maklink_lines):
                        self.maklink.add_edge(connected_lines[i].index, connected_lines[j].index)

    # -------------------------------- helper functions -------------------------------------------

    def do_lines_intersect(self, line1: Line, line2: Line):
        def orientation(line: Line, point):
            val = (line.point2.y - line.point1.y) * (point.x - line.point2.x) - (line.point2.x - line.point1.x) * (point.y - line.point2.y)
            if val == 0:
                return 0
            return 1 if val > 0 else 2
        def colinear_overlap(line_1: Line, line_2: Line):
            if min([line_1.point1.x,line_1.point2.x])<max([line_2.point1.x,line_2.point2.x]) and max([line_1.point1.x,line_1.point2.x])>min([line_2.point1.x,line_2.point2.x]):
                return True
            elif min([line_1.point1.y,line_1.point2.y])<max([line_2.point1.y,line_2.point2.y]) and max([line_1.point1.y,line_1.point2.y])>min([line_2.point1.y,line_2.point2.y]):
                return True
            else:
                return False 
        o1 = orientation(line1, line2.point1)
        o2 = orientation(line1, line2.point2)
        o3 = orientation(line2, line1.point1)
        o4 = orientation(line2, line1.point2)

        # if point orientations are all different - OR - all point orientations are 0 and lines overlap
        if (o1 != o2 and o3 != o4 and all(o != 0 for o in [o1,o2,o3,o4])) or (all(o == 0 for o in [o1,o2,o3,o4]) and colinear_overlap(line1, line2)) or (sum([1 for o in [o1,o2,o3,o4] if o==0])==1 and ((o1 != o2 and o1!=0 and o2!=0) or (o3 != o4 and o3!=0 and o4!=0))):
            return True
        return False

    def is_within_bounds(self, point: Point):
        return 0 <= point.x <= self.x_measure and 0 <= point.y <= self.y_measure

    def is_on_bounds(self, point: Point):
        return 0.0 == point.x or point.x == self.x_measure or 0.0 == point.y or point.y == self.y_measure

    def is_point_within_obstacles(self, point):
        for obstacle in self.obstacles:
            if self.is_point_inside_triangle(point, obstacle.vertices[0], obstacle.vertices[1], obstacle.vertices[2]):
                return True
        return False

    def is_point_inside_triangle(self, p: Point, a, b, c):
        def sign(p1, p2, p3):
            return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)

        d1 = sign(p, a, b)
        d2 = sign(p, b, c)
        d3 = sign(p, c, a)

        has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
        has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

        return not (has_neg and has_pos)

    # -------------------------------- evaluation of paths -------------------------------------

    def calculate_path_length(self, maklink_values, path_start=0, path_end=-1):
        path_length = 0
        if path_end==-1:
            path_end = len(self.maklink.path)-1
        for i in range(path_start, path_end):
            if i == 0:
                path_length += self.maklink.calculate_distance(self.maklink.start_point,self.maklink.maklink_lines[self.maklink.path[1]].point(maklink_values[0]))
            elif i==path_end-1:
                path_length += self.maklink.calculate_distance(self.maklink.maklink_lines[self.maklink.path[i]].point(maklink_values[i-1]),self.maklink.end_point)
            else:
                path_length += self.maklink.calculate_distance(self.maklink.maklink_lines[self.maklink.path[i]].point(maklink_values[i-1]),self.maklink.maklink_lines[self.maklink.path[i+1]].point(maklink_values[i]))
        
        return path_length
    
# =================================== visualization ============================================

    def visualize_environment(self, solution=None, draw_dijkstra_path = False, show_or_save = False, filename="test.png"):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.x_measure)
        ax.set_ylim(0, self.y_measure)

        for obstacle in self.obstacles:
            x_vals = [vertex.x for vertex in obstacle.vertices + [obstacle.vertices[0]]]
            y_vals = [vertex.y for vertex in obstacle.vertices + [obstacle.vertices[0]]]
            ax.fill(x_vals, y_vals, color='b', alpha=0.5)  # Use the alpha parameter to control transparency

        #visualize maklink intersections
        for i in range(self.maklink.size):
            plt.plot([self.maklink.maklink_lines[i].point1.x, self.maklink.maklink_lines[i].point2.x],
                     [self.maklink.maklink_lines[i].point1.y, self.maklink.maklink_lines[i].point2.y], linestyle='dashed', color='grey')
            
        # Visualize connections between Maklink lines
        for i in range(self.maklink.size):
            for j in range(i + 1, self.maklink.size):
                if self.maklink.adjacency_matrix[i][j]:
                    # Get the vertice parameter for each line

                    # Interpolate the points based on the vertice parameter
                    connection_point1 = self.maklink.maklink_lines[i].point()
                    connection_point2 = self.maklink.maklink_lines[j].point()

                    # Plot the connection line in green
                    plt.plot([connection_point1.x, connection_point2.x], [connection_point1.y, connection_point2.y], 'g-')

        plt.plot(self.maklink.start_point.x, self.maklink.start_point.y, "og", markersize=10)
        plt.plot(self.maklink.end_point.x, self.maklink.end_point.y, "or", markersize=10)

        # visualize start and end point connections
        for i in range(self.maklink.size):
            if self.maklink.adjacency_matrix[self.maklink.size][i]:
                connection_point = self.maklink.maklink_lines[i].point()
                plt.plot([self.maklink.start_point.x, connection_point.x], [self.maklink.start_point.y, connection_point.y],'g-')
            if self.maklink.adjacency_matrix[self.maklink.size+1][i]:
                connection_point = self.maklink.maklink_lines[i].point()
                plt.plot([self.maklink.end_point.x, connection_point.x], [self.maklink.end_point.y, connection_point.y],'g-')
        
        if solution is None:
            solution = [0.5] * (len(self.maklink.path)-2)

        # visualize path
        if not self.maklink.path is None:
            if draw_dijkstra_path:
                connection_point = self.maklink.maklink_lines[self.maklink.path[1]].point()
                plt.plot([self.maklink.start_point.x, connection_point.x], [self.maklink.start_point.y, connection_point.y],'k-')
                connection_point = self.maklink.maklink_lines[self.maklink.path[-2]].point()
                plt.plot([self.maklink.end_point.x, connection_point.x], [self.maklink.end_point.y, connection_point.y],'k-')
                for i in range(len(self.maklink.path)-3):
                    connection_point1 = self.maklink.maklink_lines[self.maklink.path[i+1]].point()
                    connection_point2 = self.maklink.maklink_lines[self.maklink.path[i+2]].point()
                    plt.plot([connection_point1.x, connection_point2.x], [connection_point1.y, connection_point2.y],'k-')

            connection_point = self.maklink.maklink_lines[self.maklink.path[1]].point(solution[0])
            plt.plot([self.maklink.start_point.x, connection_point.x], [self.maklink.start_point.y, connection_point.y],'r-')
            connection_point = self.maklink.maklink_lines[self.maklink.path[-2]].point(solution[-1])
            plt.plot([self.maklink.end_point.x, connection_point.x], [self.maklink.end_point.y, connection_point.y],'r-')
            for i in range(len(self.maklink.path)-3):
                connection_point1 = self.maklink.maklink_lines[self.maklink.path[i+1]].point(solution[i])
                connection_point2 = self.maklink.maklink_lines[self.maklink.path[i+2]].point(solution[i+1])
                plt.plot([connection_point1.x, connection_point2.x], [connection_point1.y, connection_point2.y],'r-')

        plt.title("Maklink Motion Planning with Genetic Path Optimization")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")

        if show_or_save:
            plt.savefig(filename)
        else:
            plt.show()

# ==================================== example case ===========================================

def load_and_create_environment(yaml_file: str):
    with open(os.path.abspath(yaml_file), 'r') as file:
        environments = yaml.safe_load(file)

    environment_data = environments[0]
    env_params = environment_data['environment']
    x = env_params['x']
    y = env_params['y']
    start_point = Point(env_params['start_point']['x'], env_params['start_point']['y'])
    end_point = Point(env_params['end_point']['x'], env_params['end_point']['y'])
    obstacles = [
        [Point(vertex['x'], vertex['y']) for vertex in obstacle]
        for obstacle in env_params['obstacles']
    ]
    env = Environment(x, y)
    for obstacle in obstacles:
        env.add_obstacle(obstacle)
    # Calculating Maklink graph
    env.calculate_maklink()
    env.add_start_point(start_point)
    env.add_end_point(end_point)
    env.maklink.dijkstra()
    env.visualize_environment(draw_dijkstra_path=True)

def example_case():
    env = Environment(10.0, 10.0)

    # Adding obstacles
    env.add_obstacle([Point(1.0, 1.0), Point(2.0, 1.0), Point(1.5, 2.0)])
    env.add_obstacle([Point(8.0, 8.0), Point(9.0, 8.0), Point(8.5, 9.0)])
    env.add_obstacle([Point(4.0, 4.0), Point(8.0, 4.0), Point(6.0, 5.0)])

    # Calculating Maklink graph
    env.calculate_maklink()

    env.add_start_point(Point(0.5,0.5))
    env.add_end_point(Point(9.5,9.5))

    env.maklink.dijkstra()

    print("Evaluation: ", env.calculate_path_length([0.5]*(len(env.maklink.path)-2)))

    # Visualizing environment with Maklink graph
    env.visualize_environment()

# ==================================== main ==================================================

if __name__ == "__main__":
    #example_case()
    load_and_create_environment("environment4.yaml")