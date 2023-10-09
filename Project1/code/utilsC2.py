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

def calculateSpeed(actual, target, heading):
    x, y = actual
    target_x, target_y = target
    
    if heading == 0:
        return 0.15 if abs(target_x - x) > 0.5 else 0.05
    
    if heading == 2:
        return 0.15 if abs(target_x - x) > 0.5 else 0.05
    
    aux = abs(target_x - x)
    aux2 = abs(target_y - y)
    return 0.15 if sqrt(pow(aux, 2) + pow(aux2, 2)) > 0.5 else 0.05


def calculateError(actual, target, previous_target):
    x, y = actual
    target_x, target_y = target
    ptarget_x, ptarget_y = previous_target

    return 0.01 * (atan2(target_y - y, target_x - x) - atan2(target_x - ptarget_x, target_y - ptarget_y))


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