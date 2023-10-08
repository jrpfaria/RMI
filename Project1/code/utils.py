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
    ones_indices = [i for i, bit in enumerate(pattern) if bit == x]

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