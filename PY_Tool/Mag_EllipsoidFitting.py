import os
import math
import matplotlib.pyplot as plt
from scipy.linalg import sqrtm
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

def create_plot(data:list, data2:list, name:str, xlabel:str, ylabel:str, color:str):
    fig_tmp = plt.figure(figsize=(5, 3))
    plt.plot(data, color=color, linestyle='-')
    if len(data) != 0:
        plt.plot(data2, color='blue', linestyle='-')
    plt.title(name, fontsize=16)
    plt.xlabel(xlabel, fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.tight_layout()
    return fig_tmp

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

for d in mag_data:
    time_stamp_list.append(int(d[0]))
    mag_x_org.append(float(d[1]))
    mag_y_org.append(float(d[2]))
    mag_z_org.append(float(d[3]))
diff_arr = np.diff(np.array(time_stamp_list)).tolist()

# show mag data time stamp diff
create_plot(diff_arr, [], 'time diff', 'sample point', 'ms diff', 'blue')

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
    # delete backward
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

data_size = len(mag_x)

# a1x^2 + a2y^2 + a3z^2 + a4xy + a5xz + a6yz + a7x + a8y + a9z = 1
# x^2, y^2, z^2, xy, xz, yz, x, y, z
D = np.array([(mag_x[i]**2, mag_y[i]**2, mag_z[i]**2, 2*mag_y[i]*mag_z[i], 2*mag_x[i]*mag_z[i], 2*mag_x[i]*mag_y[i], 2*mag_x[i], 2*mag_y[i], 2*mag_z[i], 1) for i in range(data_size)])

S = D.T @ D

# separate S
S11 = S[0:6, 0:6]
S12 = S[0:6, 6:10]
S21 = S[6:10, 0:6]
S22 = S[6:10, 6:10]

print('[ S   matrix ]')
print(S)
print('[ S11 matrix ]')
print(S11)
print('[ S12 matrix ]')
print(S12)
print('[ S13 matrix ]')
print(S21)
print('[ S14 matrix ]')
print(S22)

C = [[-1, 1, 1, 0, 0, 0],
     [1, -1, 1, 0, 0, 0],
     [1, 1, -1, 0, 0, 0],
     [0, 0, 0, -4, 0, 0],
     [0, 0, 0, 0, -4, 0],
     [0, 0, 0, 0, 0, -4]]

# Least Squares
# LS = (D.T * D)^-1 * D.T * I(unit matrix)
M0 = np.linalg.inv(C) @ (S11 - S12 @ np.linalg.inv(S22) @ S12.T)

#  separate M matrix to Value and Vector matrix
values, vector = np.linalg.eig(M0)
print('[ values ]')
print(values)
print('[ vector ]')
print(vector)

# find max value and index in diagonal matrix
max_val = np.max(np.diag(values))
max_index = np.argmax(np.diag(values))
print('[ max value ]')
print(max_val)
print('[ max index ]')
print(max_index)

v1 = vector[:,max_index]
print('[ v1 ]')
print(v1)
if v1[0] < 0:
    v1 = -v1

v2 = -np.linalg.pinv(S22) @ S12.T @ v1

# create M(3 x 3) Positive Definite Matrix
# M = [ a1,   a4/2, a5/2,
#       a4/2, a2,   a6/2, 
#       a5/2, a6/2, a3 ]
M = np.array([[v1[0],   v1[3]/2, v1[4]/2],
              [v1[3]/2, v1[1],   v1[5]/2],
              [v1[4]/2, v1[5]/2, v1[2]]])
n = v2[0:3]
d = v2[3]

print('[ M matrix ]')
print(M)

# get center
center = -np.linalg.inv(M) @ n

print('[ center ]')
print(center)

# get soft iron matrix
soft_iron_matrix = sqrtm(M)

print('[ soft iron matrix ]')
print(soft_iron_matrix)

# fix mag data
mag_org = np.array([(mag_x_org[i], mag_y_org[i], mag_z_org[i]) for i in range(data_size)])
mag_shift = mag_org - center
mag_fix = np.array([(soft_iron_matrix @ mag_shift[i]) for i in range(data_size)])

mag_shift_x = [m_d[0] for m_d in mag_shift]
mag_shift_y = [m_d[1] for m_d in mag_shift]
mag_shift_z = [m_d[2] for m_d in mag_shift]

mag_fix_x = [m_d[0] for m_d in mag_fix]
mag_fix_y = [m_d[1] for m_d in mag_fix]
mag_fix_z = [m_d[2] for m_d in mag_fix]

# plot raw and fix data in same figure
create_plot(mag_x_org, mag_shift_x, 'Mag X Shift', 'Sample point', 'X', 'red')
create_plot(mag_y_org, mag_shift_y, 'Mag Y Shift', 'Sample point', 'Y', 'red')
create_plot(mag_z_org, mag_shift_z, 'Mag Z Shift', 'Sample point', 'Z', 'red')

create_plot(mag_x_org, mag_fix_x, 'Mag X fix', 'Sample point', 'X', 'red')
create_plot(mag_y_org, mag_fix_y, 'Mag Y fix', 'Sample point', 'Y', 'red')
create_plot(mag_z_org, mag_fix_z, 'Mag Z fix', 'Sample point', 'Z', 'red')

fig = plt.figure(figsize=(6, 4))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
ax.set_zlim(-500, 500) 

ax.set_title('mag figure', fontsize=15)
ax.set_xlabel('X', fontsize=12)
ax.set_ylabel('Y', fontsize=12)
ax.set_zlabel('Z', fontsize=12)
scatter = ax.scatter(mag_fix_x, mag_fix_y, mag_fix_z, c='blue')
scatter = ax.scatter(mag_x_org, mag_y_org, mag_z_org, c='red')
plt.tight_layout()

plt.show()

# /Users/bit8/Desktop/develop/TDrone/TDrone_MCU_App/PY_Tool/mag_1.txt