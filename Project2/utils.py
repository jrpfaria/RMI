import math
from math import cos, sin, pi, degrees, radians

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

def center_of_mass(pattern, step = 1, base = 3, x = '1'):
    total_ones = pattern.count(x)
    if total_ones == 0:
        return base

    center_of_mass = sum(i * step for i, bit in enumerate(pattern) if bit == x) / total_ones

    return center_of_mass

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

def fixate_coordinates(coordinates, theta, compass, k = 2, delta = 15):
    x, y = coordinates

    delta = radians(delta)
    angle = theta if (abs(theta - compass) < k) else compass

    if (-delta) < angle < (delta) or angle > (pi - delta) or angle < (-pi + delta):
        y = find_closest_even(y)
     
    if (pi/2 - delta) < angle < (pi/2 + delta) or (-pi/2 - delta) < angle < (-pi/2 + delta):
        x = find_closest_even(x)
    
    return (x, y)


def find_closest_even(number):
    rounded_number = round(number)

    if rounded_number % 2 == 0:
        return rounded_number

    closest_even_number = rounded_number + 1 if rounded_number % 2 != 0 else rounded_number - 1

    return closest_even_number