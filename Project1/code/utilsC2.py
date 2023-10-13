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


def calculateError(actual, target):
    x, y = actual
    target_x, target_y = target

    return 0.01 * atan2(target_y - y, target_x - x)

def getRotation(line_history):
    for line in line_history[-3:]:
        if '1' in line[5:]:
            return True
    return False

def evaluateLineHistory(lineHistory):
    paths = []
    paths.extend(checkCenter(lineHistory[-1]))
    paths.extend(checkSides(lineHistory))
    return paths

def checkSides(lineHistory):
    paths = []
    hl = hr = 0

    for i in range(len(lineHistory)-1):
        if '1' in lineHistory[i][0:2]:
            if '1' in lineHistory[i][0]:
                if '1' in lineHistory[i][1]:
                    hl += 1
                elif '0' in lineHistory[i][1]:
                    paths.append('lh')
        if '1' in lineHistory[i][5:]:
            if '1' in lineHistory[i][6]:
                if '1' in lineHistory[i][5]:
                    hr += 1
                elif '0' in lineHistory[i][5]:
                    paths.append('rh')
    
    if hl > 2:
        paths.append('hl')
    if hr > 2:
        paths.append('hr')
    
    return paths


def checkCenter(line):
    paths = []
    if '1' in line[2:5]:
        paths.append('fwd')
    if '1' in line[0:2]:
        paths.append('sl')
    if '1' in line[5:]:
        paths.append('sr')

    return paths