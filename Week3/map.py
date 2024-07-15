import numpy as np
import matplotlib.pyplot as plt
import math


vals = np.zeros((100,100))


def calcAngle(tar, theta):
    err_ang = tar + theta 
    if abs(err_ang) > np.pi:
        err_ang = abs(err_ang) - 2*np.pi
    return err_ang

def checkAng(theta, ang_to_cell):
    if theta > np.pi/4 and theta < np.pi/4*3:
        return ang_to_cell >= calcAngle(theta, np.pi/4 - np.pi/100) or ang_to_cell < calcAngle(theta, np.pi/4*3)
    else: 
        return ang_to_cell > calcAngle(theta, np.pi/4 - np.pi/100) and ang_to_cell < calcAngle(theta, np.pi/4 * 3)
    

def updateMap(x, y, theta, data):
    minrange = calcAngle(theta, np.pi/4)
    rays = []
    ray_ang = np.pi/100
    for i in range(50):
        rays.append(calcAngle(minrange, i*ray_ang))
    for i in range(100):
        for j in range(100):
            cell = (i/10 - 5, j/10 - 5)
            #if occuping cell, add large positive
            if abs(x - cell[0]) <= 0.05 and abs(y - cell[1]) <= 0.05:
                vals[i,j] += 10

            ang_to_cell = np.arctan2(cell[1] - y, cell[0] - x)
            dist = ((cell[0] - x)**2+(cell[1] - y)**2)**0.5
            #if cell is in fov
            if dist < 2 and checkAng(theta, ang_to_cell):
                raydist = [x-ang_to_cell for x in rays]
                raydist = [abs(x) for x in raydist]
                ray = np.argmin(raydist)
                if float(data[ray]) != 2.0:
                    if float(data[ray]) > dist:
                        vals[i,j] += 2.2
                    elif abs(float(data[ray]) - dist) <= 0.05:
                        vals[i,j] -= 2.2





with open('OGM_Dataset.txt', 'r') as file:
    for line in file:
        data = line.split(',')
        x, y, theta = data[0:3]
        x = float(x)
        y = float(y)
        theta = float(theta)
        data = data[3:]
        updateMap(x,y,theta,data)

map = np.zeros((100,100))
for i in range(100):
    for j in range(100):
        map[i,j] = 1 - 1/(1 + math.exp(vals[i,j]))

plt.imshow(map,cmap='gray', vmin=0, vmax=1)
plt.show()