from math import sqrt, atan2, radians, pi, cos, sin

def shift_rotate_list(lst, shift):
    shift %= len(lst)
    return lst[-shift:] + lst[:-shift]

def write_map_to_file(mapa, output = "map.out"):
    with open(output, "w") as file:
        file.write(get_map_string(mapa))

def get_map_string(mapa):
    return "\n".join(["".join(row) for row in mapa])

def print_map(mapa):
    print(get_map_string(mapa), end="\r")
    
def get_beacon_path_string(path):
    return "\n".join([f"{str(Node.coordinates[0])} {str(Node.coordinates[1])}" for Node in path])

def print_beacon_path(path):
    print(get_beacon_path_string(path), end="\r")

def write_beacon_path_to_file(path, output = "plan.out"):
    with open(output, "w") as file:
        file.write(get_beacon_path_string(path))

def print_sensor_readings(line):
    print("".join(line))

def euclidean_distance(current, target):
    x1, y1 = current.coordinates
    x2, y2 = target.coordinates

    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

def manhattan_distance(current, target):
    x1, y1 = current
    x2, y2 = target.coordinates

    return abs(x2 - x1) + abs(y2 - y1)

def calculateError(actual, target, compass):
    x, y = actual.coordinates
    target_x, target_y = target.coordinates

    angle_to_target = atan2(target_y - y, target_x - x)
    angle_difference = angle_to_target - radians(compass)

    while angle_difference < -pi:
        angle_difference += 2 * pi
    while angle_difference > pi:
        angle_difference -= 2 * pi
        
    return 0.8 * angle_difference

def evaluateLineHistory(lineHistory, target):
    paths = []
    paths.extend(checkCenter(lineHistory[-1], target))
    paths.extend(checkSides(lineHistory[0:-1], target, paths))
    return list(set(paths))

def checkSides(lineHistory, target, paths):
    hl = hr = 0

    aux = 0 if paths else 1

    for i in range(len(lineHistory)-1):
        line, positions = lineHistory[i]
        next_line, next_positions = lineHistory[i+1]

        # Consider sensor position
        s0_dist = manhattan_distance(positions[0], target) - 0.24
        s1_dist = manhattan_distance(positions[1], target) - 0.16 
        s5_dist = manhattan_distance(positions[5], target) - 0.16
        s6_dist = manhattan_distance(positions[6], target) - 0.24

        ns0_dist = manhattan_distance(next_positions[0], target) - 0.24
        ns1_dist = manhattan_distance(next_positions[1], target) - 0.16 
        ns5_dist = manhattan_distance(next_positions[5], target) - 0.16
        ns6_dist = manhattan_distance(next_positions[6], target) - 0.24

        if '1' in line[2:5]:
            if '1' in line[0:2]:
                if '0' not in line[0:2]:
                    if s0_dist <= 0.1 and s1_dist <= 0.1:
                        hl += 1
                    if next_line[0] == '0' and next_line[1] == '1':
                        paths.append('lh')
                if line[1] == '0' and line[0:2] != next_line[0:2] and '1' in next_line[0:2] and ns1_dist > 0.1 and '0' not in line [2:5]:
                    paths.append('lh')

            if '1' in line[5:]:
                if '0' not in line[5:]:
                    if s6_dist <= 0.1 and s5_dist <= 0.1:
                        hr += 1
                    if next_line[6] == '0' and next_line[5] == '1':
                        paths.append('rh')
                if line[5] == '0' and line[5:] != next_line[5:] and '1' in next_line[5:] and ns5_dist > 0.1 and '0' not in line [2:5]:
                    paths.append('rh')

    line = lineHistory[-1][0]
    positions = lineHistory[-1][1]
    s0_dist = manhattan_distance(positions[0], target) - 0.24
    s1_dist = manhattan_distance(positions[1], target) - 0.16
    s5_dist = manhattan_distance(positions[5], target) - 0.16
    s6_dist = manhattan_distance(positions[6], target) - 0.24
    
    if line.count('1') == 4 and aux:
        if '1' in line[0:2] and s0_dist > 0.1 and s1_dist > 0.1:
            paths.append('sl')
        if '1' in line[5:] and s6_dist > 0.1 and s5_dist > 0.1:
            paths.append('sr')
    
    if hl > 1:
        paths.append('hl')
    elif hl == 1 and 'lh' not in paths and 'sl' not in paths:
        paths.append('hl')

    if hr > 1:
        paths.append('hr')
    elif hr == 1 and 'rh' not in paths and 'sr' not in paths:
        paths.append('hr')

    if paths == []:
        if '0' not in line[2:-1]:
            paths.append('hr')
        if '0' not in line[0:5]:
            paths.append('hl')

    return paths
    
def checkCenter(lineHistory, target):
    paths = []

    line, position = lineHistory
    
    s0_dist = manhattan_distance(position[0], target) - 0.24
    s1_dist = manhattan_distance(position[1], target) - 0.16
    s3_dist = manhattan_distance(position[3], target)
    s5_dist = manhattan_distance(position[5], target) - 0.16
    s6_dist = manhattan_distance(position[6], target) - 0.24

    if line[3] == '1' and s3_dist > 0.1:
        paths.append('fwd')
    
    if '1' in line[0:1] and s0_dist > 0.1 and s1_dist > 0.1:
        paths.append('sl')
    if '1' in line[5:] and s5_dist > 0.1 and s6_dist > 0.1:
        paths.append('sr')

    return paths

def pickPath(paths, prev_target, target):  
    unknowns = []
    x, y = target.coordinates
    px, py = prev_target.coordinates

    dx = x - px
    dy = y - py

    if 'fwd' in paths:
        unknowns.append((x + dx, y + dy, 0))
    
    if 'sr' in paths:
        # Soft right (-45º)
        if dx == 0 and dy > 0:
            unknowns.append((x + 2, y + 2, 1))
        elif dx == 0 and dy < 0:
            unknowns.append((x - 2, y - 2, 1))
        elif dx > 0 and dy == 0:
            unknowns.append((x + 2, y - 2, 1))
        elif dx < 0 and dy == 0:
            unknowns.append((x - 2, y + 2, 1))
        elif dx > 0 and dy < 0:
            unknowns.append((x, y - 2, 1))
        elif dx < 0 and dy > 0:
            unknowns.append((x, y + 2, 1))
        elif dx > 0 and dy > 0:
            unknowns.append((x + 2, y, 1))
        elif dx < 0 and dy < 0:
            unknowns.append((x - 2, y, 1))

    if 'sl' in paths:
        # Soft left (45º)     
        if dx == 0 and dy > 0:
            unknowns.append((x - 2, y + 2, 1))
        elif dx == 0 and dy < 0:
            unknowns.append((x + 2, y - 2, 1))
        elif dx > 0 and dy == 0:
            unknowns.append((x + 2, y + 2, 1))
        elif dx < 0 and dy == 0:
            unknowns.append((x - 2, y - 2, 1))
        elif dx > 0 and dy < 0:
            unknowns.append((x + 2, y, 1))
        elif dx < 0 and dy > 0:
            unknowns.append((x - 2, y, 1))
        elif dx > 0 and dy > 0:
            unknowns.append((x, y + 2, 1))
        elif dx < 0 and dy < 0:
            unknowns.append((x, y - 2, 1))

    if 'hr' in paths:
        # Handle hard right (-90º)
        if dx == 0 and dy > 0:
            unknowns.append((x + 2, y, 2))
        elif dx == 0 and dy < 0:
            unknowns.append((x - 2, y, 2))
        elif dx > 0 and dy == 0:
            unknowns.append((x, y - 2, 2))
        elif dx < 0 and dy == 0:
            unknowns.append((x, y + 2, 2))
        elif dx > 0 and dy < 0:
            unknowns.append((x - 2, y - 2, 2))
        elif dx < 0 and dy > 0:
            unknowns.append((x + 2, y + 2, 2))
        elif dx > 0 and dy > 0:
            unknowns.append((x + 2, y - 2, 2))
        elif dx < 0 and dy < 0:
            unknowns.append((x - 2, y + 2, 2))

    if 'hl' in paths:
        # Handle hard left (90º)
        if dx == 0 and dy > 0:
            unknowns.append((x - 2, y, 2))
        elif dx == 0 and dy < 0:
            unknowns.append((x + 2, y, 2))
        elif dx > 0 and dy == 0:
            unknowns.append((x, y + 2, 2))
        elif dx < 0 and dy == 0:
            unknowns.append((x, y - 2, 2))
        elif dx > 0 and dy < 0:
            unknowns.append((x + 2, y + 2, 2))
        elif dx < 0 and dy > 0:
            unknowns.append((x - 2, y - 2, 2))
        elif dx > 0 and dy > 0:
            unknowns.append((x - 2, y + 2, 2))
        elif dx < 0 and dy < 0:
            unknowns.append((x + 2, y - 2, 2))

    if 'rh' in paths:
        # Right hook (-135º)
        if dx == 0 and dy > 0:
            unknowns.append((x + 2, y - 2, 3))
        elif dx == 0 and dy < 0:
            unknowns.append((x - 2, y + 2, 3))
        elif dx > 0 and dy == 0:
            unknowns.append((x - 2, y - 2, 3))
        elif dx < 0 and dy == 0:
            unknowns.append((x + 2, y + 2, 3))
        elif dx > 0 and dy < 0:
            unknowns.append((x - 2, y, 3))
        elif dx < 0 and dy > 0:
            unknowns.append((x + 2, y, 3))
        elif dx > 0 and dy > 0:
            unknowns.append((x, y - 2, 3))
        elif dx < 0 and dy < 0:
            unknowns.append((x, y + 2, 3))

    if 'lh' in paths:
        # Left hook (135º)
        if dx == 0 and dy > 0:
            unknowns.append((x - 2, y - 2, 3))
        elif dx == 0 and dy < 0:
            unknowns.append((x + 2, y + 2, 3))
        elif dx > 0 and dy == 0:
            unknowns.append((x - 2, y + 2, 3))
        elif dx < 0 and dy == 0:
            unknowns.append((x + 2, y - 2, 3))
        elif dx > 0 and dy < 0:
            unknowns.append((x, y + 2, 3))
        elif dx < 0 and dy > 0:
            unknowns.append((x, y - 2, 3))
        elif dx > 0 and dy > 0:
            unknowns.append((x - 2, y, 3))
        elif dx < 0 and dy < 0:
            unknowns.append((x + 2, y, 3))

    return unknowns

def addToMap(paths, c2_map, map_start, prev_target, target):
    mx, my = map_start
    x, y = target.coordinates
    px, py = prev_target.coordinates

    # y - line
    # x - column

    dx = x - px # column
    dy = y - py # row

    cx = mx + x
    cy = my - y

    # y subir -> -1
    # y descer -> +1
    # x subit -> +1
    # x descer -> -1

    if 'fwd' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy - 1][cx] = "|"
        elif dx == 0 and dy < 0:
            c2_map[cy + 1][cx] = "|"
        elif dx > 0 and dy == 0:
            c2_map[cy][cx + 1] = "-"
        elif dx < 0 and dy == 0:
            c2_map[cy][cx - 1] = "-"
        elif dx > 0 and dy > 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx < 0 and dy > 0:
            c2_map[cy - 1][cx - 1] = "\\"
        elif dx > 0 and dy < 0:
            c2_map[cy + 1][cx + 1] = "\\"
        elif dx < 0 and dy < 0:
            c2_map[cy + 1][cx - 1] = "/"
    
    if 'sr' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx == 0 and dy < 0:
            c2_map[cy + 1][cx - 1] = "/"
        elif dx > 0 and dy == 0:
            c2_map[cy + 1][cx + 1] = "\\"
        elif dx < 0 and dy == 0:
            c2_map[cy - 1][cx - 1] = "\\"
        elif dx > 0 and dy > 0:
            c2_map[cy][cx + 1] = "-"
        elif dx < 0 and dy > 0:
            c2_map[cy - 1][cx] = "|"
        elif dx > 0 and dy < 0:
            c2_map[cy + 1][cx] = "|"
        elif dx < 0 and dy < 0:
            c2_map[cy][cx - 1] = "-"

    if 'sl' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy - 1][cx - 1] = "\\"
        elif dx == 0 and dy < 0:
            c2_map[cy + 1][cx + 1] = "\\"
        elif dx > 0 and dy == 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx < 0 and dy == 0:
            c2_map[cy + 1][cx - 1] = "/"
        elif dx > 0 and dy > 0:
            c2_map[cy - 1][cx] = "|"
        elif dx < 0 and dy > 0:
            c2_map[cy][cx - 1] = "-"
        elif dx > 0 and dy < 0:
            c2_map[cy][cx + 1] = "-"
        elif dx < 0 and dy < 0:
            c2_map[cy + 1][cx] = "|"

    if 'hr' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy][cx + 1] = "-"
        elif dx == 0 and dy < 0:
            c2_map[cy][cx - 1] = "-"
        elif dx > 0 and dy == 0:
            c2_map[cy + 1][cx] = "|"
        elif dx < 0 and dy == 0:
            c2_map[cy - 1][cx] = "|"
        elif dx > 0 and dy > 0:
            c2_map[cy + 1][cx + 1] = "\\"
        elif dx < 0 and dy > 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx > 0 and dy < 0:
            c2_map[cy + 1][cx - 1] = "/"
        elif dx < 0 and dy < 0:
            c2_map[cy - 1][cx - 1] = "\\"

    if 'hl' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy][cx - 1] = "-"
        elif dx == 0 and dy < 0:
            c2_map[cy][cx + 1] = "-"
        elif dx > 0 and dy == 0:
            c2_map[cy - 1][cx] = "|"
        elif dx < 0 and dy == 0:
            c2_map[cy + 1][cx] = "|"
        elif dx > 0 and dy > 0:
            c2_map[cy - 1][cx - 1] = "\\"
        elif dx < 0 and dy > 0:
            c2_map[cy + 1][cx - 1] = "/"
        elif dx > 0 and dy < 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx < 0 and dy < 0:
            c2_map[cy + 1][cx + 1] = "\\"

    if 'rh' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy + 1][cx + 1] = "\\"
        elif dx == 0 and dy < 0:
            c2_map[cy - 1][cx - 1] = "\\"
        elif dx > 0 and dy == 0:
            c2_map[cy + 1][cx - 1] = "/"
        elif dx < 0 and dy == 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx > 0 and dy > 0:
            c2_map[cy + 1][cx] = "|"
        elif dx < 0 and dy > 0:
            c2_map[cy][cx + 1] = "-"
        elif dx > 0 and dy < 0:
            c2_map[cy][cx - 1] = "-"
        elif dx < 0 and dy < 0:
            c2_map[cy - 1][cx] = "|"

    if 'lh' in paths:
        if dx == 0 and dy > 0:
            c2_map[cy + 1][cx - 1] = "/"
        elif dx == 0 and dy < 0:
            c2_map[cy - 1][cx + 1] = "/"
        elif dx > 0 and dy == 0:
            c2_map[cy - 1][cx - 1] = "\\"
        elif dx < 0 and dy == 0:
            c2_map[cy + 1][cx + 1] = "\\"
        elif dx > 0 and dy > 0:
            c2_map[cy][cx - 1] = "-"
        elif dx < 0 and dy > 0:
            c2_map[cy + 1][cx] = "|"
        elif dx > 0 and dy < 0:
            c2_map[cy - 1][cx] = "|"
        elif dx < 0 and dy < 0:
            c2_map[cy][cx + 1] = "-"

    return c2_map

def addToMapStart(line, compass, c2_map, map_start, paths):
    mx, my = map_start

    if '1' in line[2:5]:
        if -10 < compass < 10:
            c2_map[my][mx + 1] = "-"
            paths.append((2, 0))
        elif 35 < compass < 55:                    
            c2_map[my - 1][mx + 1] = "/"
            paths.append((2, 2))
        elif 80 < compass < 100:
            c2_map[my - 1][mx] = "|"
            paths.append((0, 2))
        elif 125 < compass < 145: 
            c2_map[my - 1][mx - 1] = "\\"
            paths.append((-2, +2))
        elif 170 < compass < 180 or -180 < compass < -170:
            c2_map[my][mx - 1] = "-"
            paths.append((-2, 0))
        elif -145 < compass < -125:
            c2_map[my + 1][mx - 1] = "/"
            paths.append((-2, -2))
        elif -100 < compass < -80:
            c2_map[my + 1][mx] = "|"
            paths.append((0, -2))
        elif -55 < compass < -35:
            c2_map[my + 1][mx + 1] = "\\" 
            paths.append((2, -2)) 

    return (c2_map, paths)

def calculate_sensor_positions(current_node, compass, sensor_distance=0.438, sensor_spacing=0.08):
    x, y = current_node.coordinates
    middle_sensor_x = x + sensor_distance * cos(radians(compass))
    middle_sensor_y = y + sensor_distance * sin(radians(compass))

    right_sensor_positions = []
    for i in range(1, 4):
        angle = compass - 90
        right_sensor_x = middle_sensor_x + i * sensor_spacing * cos(radians(angle))
        right_sensor_y = middle_sensor_y + i * sensor_spacing * sin(radians(angle))
        right_sensor_positions.append((right_sensor_x, right_sensor_y))

    left_sensor_positions = []
    for i in range(1, 4):
        angle = compass + 90
        left_sensor_x = middle_sensor_x + i * sensor_spacing * cos(radians(angle))
        left_sensor_y = middle_sensor_y + i * sensor_spacing * sin(radians(angle))
        left_sensor_positions.insert(0, (left_sensor_x, left_sensor_y))

    all_sensor_positions = left_sensor_positions + [(middle_sensor_x, middle_sensor_y)] + right_sensor_positions

    return all_sensor_positions
