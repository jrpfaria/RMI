from typing import List
from math import sqrt, atan2

def writeMapToFile(mapa: List[List], output: str) -> None:
    with open(output, "w") as file:
        for row in mapa:
            file.write(" ".join(row) + "\n")

def getMapAsString(mapa: List[List]) -> str:
    return "\n".join([" ".join(row) for row in mapa])

def printMap(mapa: List[List]) -> None: 
    print(getMapAsString(mapa), end="\r")

def print_sensor_readings(line):
    print("".join(line))

def euclidean_distance(coordinates, target):
    x1, y1 = coordinates
    x2, y2 = target

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


def calculateError(actual, target, previous_target):
    x, y = actual
    target_x, target_y = target
    ptarget_x, ptarget_y = previous_target

    return 0.01 * (atan2(target_y - y, target_x - x) - atan2(target_x - ptarget_x, target_y - ptarget_y))

def calculateHeading(compass):
    if -22.5 < compass < 22.5:
        return 0
    if 22.5 < compass < 67.5:
        return 1
    if 67.5 < compass < 112.5:
        return 2
    if 112.5 < compass < 157.5:
        return 3
    if 157.5 < compass <= 180 or -180 <= compass < -157.5:
        return 4
    if -157.5 < compass < -112.5:
        return -3
    if -112.5 < compass < -67.5:
        return -2
    if -67.5 < compass < -22.5:
        return -1

def changeTarget(target, heading):
    target_x, target_y = target

    if heading == 0:
        return (target_x + 2, target_y)
    
    if heading == 1:
        return (target_x + 2, target_y - 2)

    if heading == 2:
        return (target_x, target_y - 2)
    
    if heading == 3:
        return (target_x - 2, target_y - 2)
    
    if heading == -1:
        return (target_x + 2, target_y + 2)
    
    if heading == -2:
        return (target_x, target_y + 2)
    
    if heading == -3:
        return (target_x - 2, target_y + 2)
    
    if heading == -4 or heading == 4:
        return (target_x - 2, target_y)

def getRotation(line_history):
    for line in line_history[-3:]:
        if '1' in line[5:]:
            return True
    return False


def readjustToLine(line_history):
    left = right = 0
    
    for line in enumerate(line_history):
        if '1' in line[0:3]:
            left += 1
        if '1' in line[4:7]:
            right += 1

    if left > right:
        return -0.05, 0.05
    return 0.05, -0.05