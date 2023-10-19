from math import sqrt, atan2, radians, pi

def shift_rotate_list(lst, shift):
    shift %= len(lst)
    return lst[-shift:] + lst[:-shift]

def write_map_to_file(mapa, output = "mapping.out"):
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

def write_beacon_path_to_file(path, output = "planning.out"):
    with open(output, "w") as file:
        file.write(get_beacon_path_string(path))

def print_sensor_readings(line):
    print("".join(line))

def euclidean_distance(current, target):
    x1, y1 = current.coordinates
    x2, y2 = target.coordinates

    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
    
def calculateSpeed(actual, target, heading):
    x, y = actual
    target_x, target_y = target
    
    if heading == 0 or heading == 4 or heading == -4:
        return 0.1 if abs(target_x - x) > 0.5 else 0.05
    
    if heading == 2 or heading == -2:
        return 0.1 if abs(target_y - y) > 0.5 else 0.05
    
    aux = abs(target_x - x)
    aux2 = abs(target_y - y)
    return 0.1 if sqrt(pow(aux, 2) + pow(aux2, 2)) > 0.5 else 0.05

def calculateError(actual, target, compass):
    x, y = actual.coordinates
    target_x, target_y = target.coordinates

    angle_to_target = atan2(target_y - y, target_x - x)
    angle_difference = angle_to_target - radians(compass)

    while angle_difference < -pi:
        angle_difference += 2 * pi
    while angle_difference > pi:
        angle_difference -= 2 * pi
        
    return 0.420 * angle_difference

def getRotation(line_history):
    for line in line_history[-3:]:
        if '1' in line[5:]:
            return True
    return False

def evaluateLineHistory(lineHistory):
    paths = []
    paths.extend(checkCenter(lineHistory[-1]))
    paths.extend(checkSides(lineHistory, paths))
    return list(set(paths))

def checkSides(lineHistory, paths):
    hl = hr = 0

    # Check for paths on the sides
    for i in range(len(lineHistory)-1):
        # Check for paths on the left side
        if '1' in lineHistory[i][0:2]:
            # Check for hard lefts or left hooks
            if '1' in lineHistory[i][0]:
                if '1' in lineHistory[i][1]:
                    hl += 1
                elif '0' in lineHistory[i][1]:
                    paths.append('lh')

        # Check for paths on the right side
        if '1' in lineHistory[i][5:]:
            # Check for hard rights or right hooks
            if '1' in lineHistory[i][6]:
                if '1' in lineHistory[i][5]:
                    hr += 1
                elif '0' in lineHistory[i][5]:
                    paths.append('rh')
    
    # Make sure that we really got a hard turn
    if hl > 2:
        paths.append('hl')
    elif 1 <= hl <= 2 and 'sl' not in paths and 'lh' not in paths: paths.append('hl')
    
    if hr > 2:
        paths.append('hr')
    elif 1 <= hr <= 2 and 'sr' not in paths and 'rh' not in paths: paths.append('hr')
    
    return paths

def checkCenter(line):
    paths = []
    if '1' in line[3]:
        paths.append('fwd')
    if '1' in line[0]:
        paths.append('sl')
    if '1' in line[6]:
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

    if line[3] == '1':
        if compass == 0:
            c2_map[my][mx + 1] = "-"
            paths.append((2, 0))
        elif compass == 45:                    
            c2_map[my - 1][mx + 1] = "/"
            paths.append((2, 2))
        elif compass == 90:
            c2_map[my - 1][mx] = "|"
            paths.append((0, 2))
        elif compass == 135: 
            c2_map[my - 1][mx - 1] = "\\"
            paths.append((-2, -2))
        elif compass == 180 or compass == -180:
            c2_map[my][mx - 1] = "-"
            paths.append((-2, 0))
        elif compass == -135:
            c2_map[my + 1][mx - 1] = "/"
            paths.append((-2, -2))
        elif compass == -90:
            c2_map[my + 1][mx] = "|"
            paths.append((0, 2))
        elif compass == -45:
            c2_map[my + 1][mx + 1] = "\\" 
            paths.append((2, -2)) 

    return (c2_map, paths)