import numpy as np
import matplotlib.pyplot as plt


def safe_ang_add(tar, theta):
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


data = []
grid_map = []
with open('gen_map.txt', 'r') as file:
    for line in file:
        data = line.split(',')
        data = [float(x) for x in data]
        grid_map.append(data)

grid_map = np.array(grid_map)





#TODO This is temporary until odometry data is recieved
def motion_model(input, state):
    x_noise = np.random.normal(loc=0.0, scale=0.1)
    y_noise = np.random.normal(loc=0.0, scale=0.1)
    theta_noise = np.random.normal(loc=0.0, scale=np.pi/100)
    return state[0] + input[0] + x_noise, state[1] + input[1] + y_noise, safe_ang_add(safe_ang_add(state[2], input[2]),theta_noise)

def measure_model(data, state):
    x, y, theta = state
    x_coord, y_coord = coord_to_plt(x,y)
    if grid_map[min(x_coord,99),min(y_coord,99)] <= 0.6:
        return 0
    err = 0
    for i in range(len(data)):
        ray_ang = safe_ang_add(theta, np.pi/4 - i*np.pi/100)
        x_dir = 0.1*np.cos(ray_ang)
        y_dir = 0.1*np.sin(ray_ang)
        expected = 2.0
        for i in range (21):
            x_coord, y_coord = coord_to_plt(x + i * x_dir,y + i * y_dir)
            if grid_map[min(x_coord,99),min(y_coord,99)] <= 0.5:
                expected = i*0.1
                break
        noise = np.random.normal(loc=0.0, scale=0.05)
        err += abs(data[i] + noise - expected)
    # if err <= 0.2:
    #     weight = 5
    # else:
    weight = 1/err
    return weight



def mcl(prev_states, data, input, num_particles = 1000):
    particles = []
    weights = []
    for i in range(num_particles):
        p = motion_model(input ,prev_states[i])
        w = measure_model(data, p)
        particles.append(p)
        weights.append(w)
    weights = np.array(weights)
    weights = weights / weights.sum()
    samples = np.random.choice(len(particles), size=num_particles, p=weights)
    samples = [particles[x] for x in samples]
    return samples


with open('OGM_Dataset.txt', 'r') as file:
    num_part = 250
    x_pos = np.random.uniform(-5, 5, num_part)
    y_pos = np.random.uniform(-5, 5, num_part)
    theta = np.random.uniform(-np.pi, np.pi, num_part)
    prev_pose_est = list(zip(x_pos, y_pos, theta))
    prev_pose_real = (0,0,0)
    count = 0
    for line in file:
        data = line.split(',')
        x, y, theta = data[0:3]
        x = float(x)
        y = float(y)
        theta = float(theta)
        data = data[3:]
        data = [abs(float(x)) for x in data]
        
        if count % 10 == 0:
            input = (x-prev_pose_real[0],y-prev_pose_real[1],safe_ang_add(theta, -prev_pose_real[2]))
            # print(input[0],input[1], np.rad2deg(input[2]))
            prev_pose_est = mcl(prev_pose_est,data,input, num_part)
            prev_pose_real = (x,y,theta)
        if count % 50 == 0:
            plt.imshow(grid_map,cmap='gray', vmin=0, vmax=1)

            for i,j,t in prev_pose_est:
                x_plt, y_plt = coord_to_plt(i,j)
                plt.scatter(x_plt,y_plt,color='red',s=1)
            
            x_plt,y_plt = coord_to_plt(prev_pose_real[0],prev_pose_real[1])
            plt.scatter(x_plt,y_plt,color='blue',s=10)
            plt.show()
        print(count)
        count += 1
    
