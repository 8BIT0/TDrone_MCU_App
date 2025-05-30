import os
import math
import matplotlib.pyplot as plt
import numpy as np

def ThreeSigma_Check(data:list) -> list:
    if len(data) == 0:
        return []

    # step1 get average (μ)
    avg = sum(data) / len(data)

    # step2 get stander deviation (σ)
    std_dev = math.sqrt(sum((d - avg) ** 2 for d in data)/ len(data))
    
    min_val = avg - 3 * std_dev
    max_val = avg + 3 * std_dev

    # step3 check data should between the range [μ - 3σ,μ + 3σ] or else exclude the error data
    remove_list = []
    index = 0
    for d in data:
        if d < min_val and d > max_val:
            remove_list.append(index)
        index += 1

    return remove_list

def create_plot(data:list, name:str, xlabel:str, ylabel:str, color:str, marker:str, linestyle:str):
    fig_tmp = plt.figure(figsize=(8, 6))
    pass

def Load_DataFile() -> list:
    file_path = input('[ Mag data file path ]:')
    mag_data_list = []
    try:
        with open(file_path, 'r') as data:
            for line in data:
                mag_data_list.append(line[:-1].split(' '))
            return mag_data_list
    except FileNotFoundError:
        print('[ File not exist ]')
        return []
    except PermissionError:
        print('[ Access permission denied ]')
        return []
    except Exception as e:
        print(f'[ Error: {e} ]')
        return []
    
mag_data = Load_DataFile()
if len(mag_data) == 0:
    print('[ Load Mag data failed ]')
    exit()

time_stamp_list = []
mag_x_org = []
mag_y_org = []
mag_z_org = []

# /Users/bit8/Desktop/develop/TDrone/TDrone_MCU_App/PY_Tool/mag_1.txt
for d in mag_data:
    time_stamp_list.append(int(d[0]))
    mag_x_org.append(float(d[1]))
    mag_y_org.append(float(d[2]))
    mag_z_org.append(float(d[3]))
diff_arr = np.diff(np.array(time_stamp_list)).tolist()

# show mag data time stamp diff
plt.figure(figsize=(8, 6))
plt.plot(diff_arr, color='blue', marker='o', linestyle='-')
plt.title('time diff', fontsize=16)
plt.xlabel('sample point', fontsize=12)
plt.ylabel('ms diff', fontsize=12)
plt.tight_layout()

if len(mag_x_org) !=len(mag_y_org) or len(mag_y_org)!=len(mag_z_org):
    print('[ Origin Mag data size not equal ]')
    exit()

# 3 sigma check
x_remove = ThreeSigma_Check(mag_x_org)
y_remove = ThreeSigma_Check(mag_y_org)
z_remove = ThreeSigma_Check(mag_z_org)

print(f'x_remove {x_remove}')
print(f'y_remove {y_remove}')
print(f'z_remove {z_remove}')

remove_list = sorted(x_remove + y_remove + z_remove)
print(f'remove_list {remove_list}')

# trim mag data list
if len(remove_list) != 0:
    for index in sorted(remove_list, reverse=True):
        del mag_x_org[index]
        del mag_y_org[index]
        del mag_z_org[index]

mag_x = np.array(mag_x_org)
mag_y = np.array(mag_y_org)
mag_z = np.array(mag_z_org)

if len(mag_x)!=len(mag_y) or len(mag_y)!=len(mag_z):
    print('[ 3 sigma check Mag data size not equal ]')
    exit()

mag_x_mean = np.mean(mag_x)
mag_y_mean = np.mean(mag_y)
mag_z_mean = np.mean(mag_z)

print(f'[ Mag X Mean {mag_x_mean} ]')
print(f'[ Mag Y Mean {mag_y_mean} ]')
print(f'[ Mag Z Mean {mag_z_mean} ]')

if mag_x_mean == np.nan or mag_y_mean == np.nan or mag_z_mean == np.nan:
    print('[ mag data mean is NaN ]')
    exit()

data_size = len(mag_x)

x = mag_x - mag_x_mean
y = mag_y - mag_y_mean
z = mag_z - mag_z_mean

# plot new mag data

# a1x^2 + a2y^2 + a3z^2 + a4xy + a5xz + a6yz + a7x + a8y + a9z = 1
# x^2, y^2, z^2, xy, xz, yz, x, y, z
D = np.array([(x[i]**2, y[i]**2, z[i]**2, x[i]*y[i], x[i]*z[i], y[i]*z[i], x[i], y[i], z[i]) for i in range(data_size)])

# Least Squares
# LS = (D.T * D)^-1 * D.T * I(unit matrix)
a = np.linalg.pinv(D) @ np.ones(data_size)
print(a)

# create M(3 x 3) matrix
# M = [ a1,   a4/2, a5/2,
#       a4/2, a2,   a6/2, 
#       a5/2, a6/2, a3 ]
M = np.array([[a[0], a[4]/2, a[5]/2],
              [a[4]/2, a[2], a[6]/2],
              [a[5]/2, a[6]/2, a[3]]])

# center = -(1/2)[a7, a8, a9]M^-1
center = -0.5 * np.array([a[6], a[7], a[8]]) @ np.linalg.inv(M)

print(M)
print(center)

# fix mag data
# mag_fix = M^-1 * (mag - center)

plt.show()
