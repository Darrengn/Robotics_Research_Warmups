import numpy as np
import matplotlib.pyplot as plt
import math
import sys

vals = np.zeros((100,100))


def calcAngle(tar, theta):
    err_ang = tar + theta 
    if abs(err_ang) > np.pi:
        err_ang = abs(err_ang) - 2*np.pi
        if tar < 0:
            err_ang = -err_ang
    return err_ang

def plt_to_coord(x_ind:int, y_ind:int):
    return x_ind*0.1 - 5, y_ind*0.1 - 5

def coord_to_plt(x_pos, y_pos):
    return int(x_pos/0.1 + 5/0.1), int(y_pos/0.1 + 5/0.1)

def checkAng(theta, ang_to_cell):
    if theta > np.pi/4*3 or theta < -np.pi/4*3:
        return ang_to_cell <= calcAngle(theta, np.pi/4) or ang_to_cell > calcAngle(theta, -np.pi/4 + np.pi/100)
    else: 
        return ang_to_cell < calcAngle(theta, np.pi/4) and ang_to_cell > calcAngle(theta, -np.pi/4)
    

def updateMap(x, y, theta, data, count):
    minrange = calcAngle(theta, np.pi/4)
    rays = []
    ray_ang = np.pi/100
    for i in range(50):
        rays.append(calcAngle(minrange, -i*ray_ang))
    for i in range(100):
        for j in range(100):
            cell = (i/10 - 5, j/10 - 5)

            #if occuping cell, add large positive
            if abs(x - cell[0]) <= 0.05 and abs(y - cell[1]) <= 0.07:
                vals[i,j] += 5

            ang_to_cell = np.arctan2(cell[1] - y, cell[0] - x)
            dist = ((cell[0] - x)**2+(cell[1] - y)**2)**0.5
            #if cell is in fov
            if dist < 2 and checkAng(theta, ang_to_cell):
                # print(i,j,"in range:")
                raydist = [x-ang_to_cell for x in rays]
                raydist = [abs(x) for x in raydist]
                ray = np.argmin(raydist)
                if float(data[ray]) != 2.0:
                    if abs(float(data[ray]) - dist) <= 0.07:
                        vals[i,j] -= 2.2
                    elif float(data[ray]) > dist:
                        vals[i,j] += 2.2
                else:
                    vals[i,j] += 2.2
                    if i == 67 and j == 87:
                        bug = [np.rad2deg(x) for x in rays]
                        # print(ray, data[ray], bug[ray], np.rad2deg(theta), count)
"""
    Uses ray tracing
"""                          
def updateMap2(x,y,theta,data, count):
    for i in range(len(data)):
        ray_ang = calcAngle(theta, np.pi/4 - i*np.pi/100)
        x_dir = 0.1*np.cos(ray_ang)
        y_dir = 0.1*np.sin(ray_ang)
        cur_x = x
        cur_y = y
        obs_x = float(data[i])*np.cos(ray_ang) + x
        obs_y = float(data[i])*np.sin(ray_ang) + y
        
        if float(data[i]) < 2.0:
            x_ind, y_ind = coord_to_plt(obs_x,obs_y)
            vals[x_ind,y_ind] -= 2.2
            if x_ind == 40 and y_ind == 20:
                print(data[i], obs_x, obs_y, x, y, np.rad2deg(theta), np.rad2deg(ray_ang))
            while abs(cur_x - obs_x) >= 0.1 and abs(cur_y - obs_y) >= 0.1:
                vals[coord_to_plt(cur_x,cur_y)] += 2.2
                cur_x += x_dir
                cur_y += y_dir
        else:
            vals[coord_to_plt(obs_x,obs_y)] += 2.2
            while abs(cur_x - obs_x) >= 0.1 and abs(cur_y - obs_y) >= 0.1:
                vals[coord_to_plt(cur_x,cur_y)] += 2.2
                cur_x += x_dir
                cur_y += y_dir
            # vals[coord_to_plt(obs_x,obs_y)] += 2.2
            # while abs(cur_x - obs_x) >= 0.1 and abs(cur_y - obs_y) >= 0.1:
            #     vals[coord_to_plt(cur_x,cur_y)] += 2.2
            #     print(coord_to_plt(cur_x,cur_y), cur_x, cur_y)
            #     cur_x += x_dir
            #     cur_y += y_dir

count = 0
with open('OGM_Dataset.txt', 'r') as file:
    for line in file:
        data = line.split(',')
        x, y, theta = data[0:3]
        x = float(x)
        y = float(y)
        theta = float(theta)
        data = data[3:]
        nums = [abs(float(x)) for x in data]
        # if min(nums) < min_val:
        #     min_val = min(nums)
        # if count == 3500:
        updateMap2(x,y,theta,data, count)
        count += 1




for i in range(100):
    for j in range(100):
        if vals[i,j] > 100:
            vals[i,j] = 1
        else:
            vals[i,j] = 1 - 1/(1 + math.exp(vals[i,j]))

np.savetxt('gen_map.txt', vals, delimiter=',')

plt.imshow(vals,cmap='gray', vmin=0, vmax=1)
plt.show()