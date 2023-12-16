import math
from math import cos, sin, pi, degrees, radians, sqrt, atan2, floor

PATTERNS = [
    [1, 0, 0, 0, 0, 0, 0],
    [1, 1, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 0, 0, 0], 
    [0, 1, 1, 1, 0, 0, 0], 
    [0, 0, 1, 1, 1, 0, 0], 
    [0, 0, 0, 1, 1, 1, 0], 
    [0, 0, 0, 0, 1, 1, 1], 
    [0, 0, 0, 0, 0, 1, 1], 
    [0, 0, 0, 0, 0, 0, 1]
]

def calculate_out(prev_power, power):
    left_prev_power, right_prev_power = prev_power
    left_power, right_power = power

    return ((left_prev_power + left_power) / 2, (right_prev_power + right_power) / 2)

def adjust_angle(theta):
    theta = theta % (2 * pi)
    if theta > pi:
        theta -= 2 * pi
    return theta

def get_last_direction(history):
    weighted_sum = 0
    total_weight = 0

    for i, direction in enumerate(history):
        weight = 1 / (i + 1)  # Linearly decreasing weight
        weighted_sum += direction * weight
        total_weight += weight

    weighted_average = weighted_sum / total_weight
    return weighted_average

def write_map_to_file(mapa, output = "map.out"):
    with open(output, "w") as file:
        file.write(get_map_string(mapa))

def get_map_string(mapa):
    return "\n".join(["".join(row) for row in mapa])

def print_map(mapa):
    print(get_map_string(mapa), end="\r")

def print_sensor_readings(line):
    print("".join(line))

def adjust_power(last_left_power, last_right_power, left_power, right_power):
    if last_left_power == None:
        adjusted_left_power = left_power
    else:
        adjusted_left_power = 2 * left_power - last_left_power

    if last_right_power == None:
        adjusted_right_power = right_power
    else:
        adjusted_right_power = 2 * right_power - last_right_power

    return adjusted_left_power, adjusted_right_power

def get_beacon_path_string(path):
    return "\n".join([f"{str(node_x)} {str(node_y)}" for node_x, node_y in path])

def print_beacon_path(path):
    print(get_beacon_path_string(path))

def write_beacon_path_to_file(path, output = "plan.out"):
    with open(output, "w") as file:
        file.write(get_beacon_path_string(path))

def median_value(pattern, step = 1, base = 3, x = '1'):
    ones_indices = [i for i, bit in enumerate(pattern[1:6]) if bit == x]

    if not ones_indices:
        return base

    n = len(ones_indices)

    if n % 2 == 1:
        # If the number of ones is odd, return the middle index
        return ones_indices[n // 2] * step
    else:
        # If the number of ones is even, return the average of the two middle indices
        middle1 = ones_indices[n // 2 - 1]
        middle2 = ones_indices[n // 2]
        return (middle1 + middle2) / 2.0 * step

def center_of_mass(pattern, step = 1, x = '1'):
    base = get_base(len(pattern), step)
    total_ones = pattern.count(x)
    if total_ones == 0:
        return base

    center_of_mass = sum(i * step for i, bit in enumerate(pattern) if bit == x) / total_ones

    return center_of_mass - base

def angular_deviation(actual, target, compass):
    x, y = actual
    tx, ty = target

    angle_to_target = atan2(ty - y, tx - x)
    angle_difference = angle_to_target - compass

    angle_difference = adjust_angle(angle_difference)
        
    return angle_difference

def get_base(pattern_length, step = 1):
    middle_index = pattern_length // 2

    if pattern_length % 2 != 0: 
        return middle_index * step

    return (middle_index + 0.5) * step

def hamming_distance(pattern1, pattern2):
    return sum(c1 != c2 for c1, c2 in zip(pattern1, pattern2))

def pattern_matching(input_pattern, patterns):
    
    hamming_distances = [hamming_distance(input_pattern, pattern) for pattern in patterns]

    # Find the pattern with the minimum Hamming distance
    best_match_index = hamming_distances.index(min(hamming_distances))
    filtered_pattern = patterns[best_match_index]

    return filtered_pattern

def generate_patterns(length, max_consecutive_ones):
    patterns = []
    pattern = [1]
    
    patterns += [pattern.copy() + [0] * (length - len(pattern))]
    
    for i in range(length + max_consecutive_ones - 2):
        
        if len(pattern) == length:
            del pattern[-1]
            pattern.insert(0, 0)
        elif pattern[0] == 1:
            if len(pattern) >= max_consecutive_ones and sum(pattern[1:max_consecutive_ones]) == max_consecutive_ones - 1:
                pattern.insert(0, 0)
            else:
                pattern.insert(0, 1)
        else:
            pattern.insert(0, 0)
            
        patterns += [pattern.copy() + [0] * (length - len(pattern))]  
                
    return patterns

def calculate_slope(x, y):
    n = len(x)
    if n < 2:
        return None
    x_mean = sum(x) / n
    y_mean = sum(y) / n 
    numerator = sum((x[i] - x_mean) * (y[i] - y_mean) for i in range(n))
    denominator = sum((x[i] - x_mean) ** 2 for i in range(n))   
    slope = numerator / denominator
    return slope

def is_likely_ascending(data, threshold=0.9):
    if len(data) < 2:
        return False
    x = list(range(len(data)))
    slope = calculate_slope(x, data)

    if slope is not None and slope >= threshold:
        return True
    else:
        return False

def remove_outliers(binary_list):
    result = []
    for i in range(len(binary_list)):
        if binary_list[i] == '1':
            left_neighbor = binary_list[i - 1] if i > 0 else '0'
            right_neighbor = binary_list[i + 1] if i < len(binary_list) - 1 else '0'
            if left_neighbor == '1' or right_neighbor == '1':
                result.append('1')
            else:
                result.append('0')
        else:
            if (i > 0 and binary_list[i - 1] == '1') and (i < len(binary_list) - 1 and binary_list[i + 1] == '1'):
                result.append('1')
                result[-2] = '1'
            else:
                result.append('0')
    return result

def check_for_window_pattern(history):

    print(history)

    if len(history) < 2:
        if len(history) == 1:
            if history[0] == 0.08:
                return True, 'Possible Miss'
        return False, 'Hit the tip'
    
    if any(line > 0.32 for line in history):
        return True, 'Needs adjustment'
        
    avg = sum(history) / len(history)

    history = [line for line in history if abs(avg - line) < 0.16]
    
    return is_likely_ascending(history, -0.035), 'Analyzing slope'

def movement_model(out, wpow, gauss_noise = 1):
    l_out, r_out = out
    l_in, r_in = wpow
    
    l_out = ((l_in + l_out) / 2) * gauss_noise
    r_out = ((r_in + r_out) / 2) * gauss_noise

    return (l_out, r_out)

def rotation_model(power, D = 1):
    lpow, rpow = power

    # rotation is expressed in radians
    rot = (rpow - lpow) / D
    return rot

def gps_model(pos, out, theta):
    x, y = pos
    l_pow, r_pow = out
    
    lin = (l_pow + r_pow) / 2

    # theta's value is expressed in radians
    gps_x = x + lin * cos(theta)
    gps_y = y + lin * sin(theta)

    return (gps_x, gps_y)

def fixate_coordinates(coordinates, theta, delta = 15):
    x, y = coordinates

    delta = radians(delta)
    angle = theta

    if (-delta) < angle < (delta) or angle > (pi - delta) or angle < (-pi + delta):
        y = find_closest_even(y)
        return (x, y)
     
    if (pi/2 - delta) < angle < (pi/2 + delta) or (-pi/2 - delta) < angle < (-pi/2 + delta):
        x = find_closest_even(x)
        return (x, y)
    
    return (x, y)

def fixate_theta(angle, delta=15):
    delta = radians(delta)
    reference_angles = [0, pi, pi/2, -pi/2, pi/4, -3*pi/4, 3*pi/4, -pi/4]

    closest_angle = min(reference_angles, key=lambda x: abs(x - angle))

    if abs(angle - closest_angle) < delta:
        return closest_angle
    
    return angle

def find_closest_even(n):
    rounded = round(n)
    
    if rounded % 2 == 0:
        return rounded
    
    if n < 0:
        return rounded - 1 if abs(n - (rounded - 1)) < abs(n - (rounded + 1)) else rounded + 1
    return rounded - 1 if abs(n - (rounded - 1)) < abs(n - (rounded + 1)) else rounded + 1

def manhattan_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return abs(x1 - x2) + abs(y1 - y2)

def euclidian_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def is_close(p1, p2, threshold = 0.5, distance = euclidian_distance):
    return distance(p1, p2) < threshold

def is_far(p1, p2, threshold = 0.5, distance = euclidian_distance):
    return distance(p1, p2) > threshold

def sort_paths(paths):
    order = {'fwd': 0, 'sl': 1, 'sr': 2, 'hl': 3, 'hr': 4, 'lh': 5, 'rh': 6}
    return sorted(paths, key=lambda x: order[x])


def translate_paths(paths, prev_target, target):  
    unknowns = []
    x, y = target
    px, py = prev_target

    dx = x - px
    dy = y - py

    angle = round(degrees(atan2(dy, dx)))

    direction_offsets = {
        'fwd': (x + dx, y + dy),
        'sr': {
            0: (x + 2, y - 2),
            45: (x + 2, y),
            90: (x + 2, y + 2),
            135: (x, y + 2),
            180: (x - 2, y + 2),
            -45: (x, y - 2),
            -90: (x - 2, y - 2),
            -135: (x - 2, y)
        },
        'sl': {
            0: (x + 2, y + 2),
            45: (x, y + 2),
            90: (x - 2, y + 2),
            135: (x - 2, y),
            180: (x - 2, y - 2),
            -45: (x + 2, y),
            -90: (x + 2, y - 2),
            -135: (x, y - 2)
        },
        'hr': {
            0: (x, y - 2),
            45: (x + 2, y - 2),
            90: (x + 2, y),
            135: (x + 2, y + 2),
            180: (x, y + 2),
            -45: (x - 2, y - 2),
            -90: (x - 2, y),
            -135: (x - 2, y + 2)
        },
        'hl': {
            0: (x, y + 2),
            45: (x - 2, y + 2),
            90: (x - 2, y),
            135: (x - 2, y - 2),
            180: (x, y - 2),
            -45: (x + 2, y + 2),
            -90: (x + 2, y),
            -135: (x + 2, y - 2)
        },
        'rh': {
            0: (x - 2, y - 2),
            45: (x, y - 2),
            90: (x + 2, y - 2),
            135: (x + 2, y),
            180: (x + 2, y + 2),
            -45: (x - 2, y),
            -90: (x - 2, y + 2),
            -135: (x, y + 2)
        },
        'lh': {
            0: (x - 2, y + 2),  
            45: (x - 2, y),
            90: (x - 2, y - 2),  
            135: (x, y - 2),
            180: (x + 2, y - 2),    
            -45: (x, y + 2),
            -90: (x + 2, y + 2),   
            -135: (x + 2, y)
        }
    }

    translations = {}
    for path in paths:
        offset = direction_offsets[path] if path == 'fwd' else direction_offsets[path][angle] 
        translations[offset] = path
        unknowns.append(offset)

    return unknowns, translations

def general_movement_model(left_power, right_power, out, theta, coordinates):
    wpow = (left_power, right_power)
    out = calculate_out(out, wpow)
    theta += rotation_model(out)
    theta = adjust_angle(theta)
    coordinates = gps_model(coordinates, out, theta)

    return coordinates, theta

def centered_line(line, n_sensors):
    n = len(line)

    if n_sensors >= n:
        return line
    
    start_index = (n - n_sensors) // 2
    end_index = start_index + n_sensors

    centered_elements = line[start_index:end_index]

    return centered_elements


def scan_paths(lineHistory, target):
    paths = []
    # for line, _ in lineHistory:
    #     print(line)
    paths.extend(scan_center(lineHistory[-1], target))
    paths.extend(scan_sides(lineHistory[0:-1], target, paths))
    return set(paths)

def scan_sides(lineHistory, target, paths):
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
    
def scan_center(lineHistory, target):
    paths = []

    line, position = lineHistory

    s0_dist = manhattan_distance(position[0], target) - 0.24
    s1_dist = manhattan_distance(position[1], target) - 0.16
    s2_dist = manhattan_distance(position[2], target) - 0.08
    s3_dist = manhattan_distance(position[3], target)
    s4_dist = manhattan_distance(position[4], target) - 0.08
    s5_dist = manhattan_distance(position[5], target) - 0.16
    s6_dist = manhattan_distance(position[6], target) - 0.24
    
    if '1' in line[0:1] and s0_dist > 0.1 and s1_dist > 0.1:
        paths.append('sl')
    if '1' in line[5:] and s5_dist > 0.1 and s6_dist > 0.1:
        paths.append('sr')

    if line[3]=="1" and s3_dist > 0.11:
        paths.append('fwd')

    return paths

def calculate_sensor_positions(current_node, theta, compass, sensor_distance=0.438, sensor_spacing=0.08):
    x, y = current_node
    
    compass = theta if degrees(theta) % 45 == 0 else compass

    middle_sensor_x = x + sensor_distance * cos(compass)
    middle_sensor_y = y + sensor_distance * sin(compass)

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

def help_robot(coordinates, compass, delta=15):
    x, y = coordinates
    delta = radians(delta)

    x = floor(x)
    y = floor(y)

    if -delta < compass < delta:
        return (find_closest_even(x+1.1), find_closest_even(y))
    
    if -pi + delta < compass < pi - delta:
        return (find_closest_even(x-1.1), find_closest_even(y))
    
    if pi/2 - delta < compass < pi/2 + delta:
        return (find_closest_even(x), find_closest_even(y+1.1))
    
    if -pi/2 - delta < compass < -pi/2 + delta:
        return (find_closest_even(x), find_closest_even(y-1.1))

    if pi/4 - delta < compass < pi/4 + delta:
        return (find_closest_even(x+1.1), find_closest_even(y+1.1))
    
    if -3*pi/4 - delta < compass < -3*pi/4 + delta:
        return (find_closest_even(x-1.1), find_closest_even(y-1.1))
    
    if 3*pi/4 - delta < compass < 3*pi/4 + delta:
        return (find_closest_even(x-1.1), find_closest_even(y+1.1))
    
    if -pi/4 - delta < compass < -pi/4 + delta:
        return (find_closest_even(x+1.1), find_closest_even(y-1.1))

    return (0, 0)

def calculate_control_constants(coordinates, prev_target, target, MAX_SPEED):
    tx, ty = target
    px, py = prev_target

    dx = tx - px
    dy = ty - py

    angle = round(degrees(atan2(dy, dx)))

    if angle in [0, 90, 180, -90, -180]:
        return calculate_control_constants_straight(coordinates, prev_target, target, MAX_SPEED)
    return calculate_control_constants_homo(coordinates, prev_target, target, MAX_SPEED)


def calculate_control_constants_straight(coordinates, prev_target, target, MAX_SPEED):
    if is_close(coordinates, target, 0.438):
        cm_Kp = 0
        ad_Kp = 0
        base_speed = 0.10
        n_sensors = 3
    elif is_close(coordinates, target, 1.6):
        cm_Kp = 1
        ad_Kp = 0
        base_speed = 0.10
        n_sensors = 3
    elif is_close(coordinates, target, 1.9):
        cm_Kp = 4
        ad_Kp = 0
        base_speed = MAX_SPEED
        n_sensors = 7 
    else:
        cm_Kp = 0
        ad_Kp = -3
        base_speed = MAX_SPEED
        n_sensors = 3

    return cm_Kp, ad_Kp, base_speed, n_sensors

def calculate_control_constants_homo(coordinates, prev_target, target, MAX_SPEED):
    if is_close(coordinates, target, 0.438):
        cm_Kp = 0
        ad_Kp = 0
        base_speed = 0.10
        n_sensors = 3
    elif is_close(coordinates, target, 2.4):
        cm_Kp = 1
        ad_Kp = 0
        base_speed = 0.10
        n_sensors = 3
    elif is_close(coordinates, target, 2.7):
        cm_Kp = 4
        ad_Kp = 0
        base_speed = MAX_SPEED
        n_sensors = 7 
    else:
        cm_Kp = 0
        ad_Kp = -3
        base_speed = MAX_SPEED
        n_sensors = 3

    return cm_Kp, ad_Kp, base_speed, n_sensors

def calculate_control(line, n_sensors, coordinates, target, compass, cm_Kp, ad_Kp, STEP):
    evaluated_line = centered_line(line, n_sensors)
    cm_error = cm_Kp * center_of_mass(evaluated_line, STEP)
    ad_error = ad_Kp * angular_deviation(coordinates, target, compass)
    return cm_error + ad_error, cm_error, ad_error

def calculate_rotation_error(error, compass, ROTATION_SPEED, ROTATION_SLOWDOWN_THRESHOLD, MAX_SPEED):                                    
    if compass % 45 < ROTATION_SLOWDOWN_THRESHOLD:
        return max(min(error, ROTATION_SPEED), -ROTATION_SPEED)
    return max(min(error, MAX_SPEED), -MAX_SPEED)

def cap_speed(left_power, right_power, MAX_SPEED):
    left_power = min(max(left_power, -MAX_SPEED), MAX_SPEED)
    right_power = min(max(right_power, -MAX_SPEED), MAX_SPEED)
    return left_power, right_power

def update_map(paths, pmap, map_start, prev_target, target):
    mx, my = map_start
    x, y = target
    px, py = prev_target

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

    angle = round(degrees(atan2(dy, dx)))

    direction_symbols = {
        'fwd': {
            0: ((cy, cx + 1), '-'), 
            45: ((cy - 1, cx + 1), '/'),
            90: ((cy - 1, cx), '|'),
            135: ((cy - 1, cx - 1), '\\'),
            180: ((cy, cx - 1), '-'),
            -45: ((cy + 1, cx + 1), '\\'),
            -90: ((cy + 1, cx), '|'),
            -135: ((cy + 1, cx - 1), '/')
        },
        'sr': {
            0: ((cy + 1, cx + 1), '\\'),
            45: ((cy, cx + 1), '-'),
            90: ((cy - 1, cx + 1), '/'),
            135: ((cy - 1, cx), '|'),
            180: ((cy - 1, cx - 1), '\\'),
            -45: ((cy + 1, cx), '|'),
            -90: ((cy + 1, cx - 1), '/'),
            -135: ((cy, cx - 1), '-')
        },
        'sl': {
            0: ((cy - 1, cx + 1), '/'),
            45: ((cy - 1, cx), '|'),
            90: ((cy - 1, cx - 1), '\\'),
            135: ((cy, cx - 1), '-'),
            180: ((cy + 1, cx - 1), '/'),
            -45: ((cy, cx + 1), '-'),
            -90: ((cy + 1, cx + 1), '\\'),
            -135: ((cy + 1, cx), '|')
        },
        'hr': {
            0: ((cy + 1, cx), '|'),
            45: ((cy + 1, cx + 1), '\\'),
            90: ((cy, cx + 1), '-'),
            135: ((cy - 1, cx + 1), '/'),
            180: ((cy - 1, cx), '|'),
            -45: ((cy + 1, cx - 1), '/'),
            -90: ((cy, cx - 1), '-'),
            -135: ((cy - 1, cx - 1), '\\')
        },
        'hl': {
            0: ((cy - 1, cx), '|'),
            45: ((cy - 1, cx - 1), '\\'),
            90: ((cy, cx - 1), '-'),
            135: ((cy + 1, cx - 1), '/'),
            180: ((cy + 1, cx), '|'),
            -45: ((cy - 1, cx + 1), '/'),
            -90: ((cy, cx + 1), '-'),
            -135: ((cy + 1, cx + 1), '\\')
        },
        'rh': {
            0: ((cy + 1, cx - 1), '/'),
            45: ((cy + 1, cx), '|'),
            90: ((cy + 1, cx + 1), '\\'),
            135: ((cy, cx + 1), '-'),
            180: ((cy - 1, cx + 1), '/'),
            -45: ((cy, cx - 1), '-'),
            -90: ((cy - 1, cx - 1), '\\'),
            -135: ((cy - 1, cx), '|')
        },
        'lh': {
            0: ((cy - 1, cx - 1), '\\'),
            45: ((cy, cx - 1), '-'),
            90: ((cy + 1, cx - 1), '/'),
            135: ((cy + 1, cx), '|'),
            180: ((cy + 1, cx + 1), '\\'),
            -45: ((cy - 1, cx), '|'),
            -90: ((cy - 1, cx + 1), '/'),
            -135: ((cy, cx + 1), '-')
        }
    }
    
    updated = {}
    for path in paths:
        (y, x), symbol = direction_symbols[path][angle]
        pmap[y][x] = symbol
        updated[path] = (y, x)
        
    return pmap, updated

def correct_target(coordinates, compass, delta=15):
    x, y = coordinates
    delta = radians(delta)

    if -delta < compass < delta:
        return (x + 2, y)
    if -pi + delta < compass < pi - delta:
        return (x - 2, y)
    if pi/2 - delta < compass < pi/2 + delta:
        return (x, y + 2)
    if -pi/2 - delta < compass < -pi/2 + delta:
        return (x, y - 2)
    if pi/4 - delta < compass < pi/4 + delta:
        return (x + 2, y + 2)
    if -3*pi/4 - delta < compass < -3*pi/4 + delta:
        return (x - 2, y - 2)
    if 3*pi/4 - delta < compass < 3*pi/4 + delta:
        return (x - 2, y + 2)
    if -pi/4 - delta < compass < -pi/4 + delta:
        return (x + 2, y - 2)
    
    return (x, y)
    
def undo_map_update(target, pmap, translated_map_updates, map_updates):
    dt = translated_map_updates[target]
    dt_pos = map_updates[dt]
    dt_y, dt_x = dt_pos
    pmap[dt_y][dt_x] = " "
    return pmap

def find_paths_on_rotation(line, compass, coordinates, paths):
    x, y = coordinates

    if '1' in line[2:5]:
        if -10 < compass < 10:
            paths.append((x + 2, y))
        elif 35 < compass < 55:
            paths.append((x + 2, y + 2))
        elif 80 < compass < 100:
            paths.append((x, y + 2))
        elif 125 < compass < 145:
            paths.append((x - 2, y + 2))
        elif 170 < compass < 180 or -180 < compass < -170:
            paths.append((x - 2, y))
        elif -145 < compass < -125:
            paths.append((x - 2, y - 2))
        elif -100 < compass < -80:
            paths.append((x, y - 2))
        elif -55 < compass < -35:
            paths.append((x + 2, y - 2))

    return paths

def update_map_start(line, compass, pmap, map_start, target, paths):
    mx, my = map_start
    x, y = target

    tx = mx + x
    ty = my - y

    if '1' in line[2:5]:
        if -10 <= compass <= 10:
            pmap[ty][tx + 1] = "-"
            paths.append((x + 2, y))
        elif 35 <= compass <= 55:                    
            pmap[ty - 1][tx + 1] = "/"
            paths.append((x + 2, y + 2))
        elif 80 <= compass <= 100:
            pmap[ty - 1][tx] = "|"
            paths.append((x, y + 2))
        elif 125 <= compass <= 145: 
            pmap[ty - 1][tx - 1] = "\\"
            paths.append((x - 2, y + 2))
        elif 170 <= compass <= 180 or -180 <= compass <= -170:
            pmap[ty][tx - 1] = "-"
            paths.append((x - 2, y))
        elif -145 <= compass <= -125:
            pmap[ty + 1][tx - 1] = "/"
            paths.append((x - 2, y - 2))
        elif -100 <= compass <= -80:
            pmap[ty + 1][tx] = "|"
            paths.append((x, y - 2))
        elif -55 <= compass <= -35:
            pmap[ty + 1][tx + 1] = "\\" 
            paths.append((x + 2, y - 2))

    return (pmap, paths)