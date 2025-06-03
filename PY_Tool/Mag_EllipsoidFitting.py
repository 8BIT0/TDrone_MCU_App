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
D = np.array([(mag_x[i]**2, mag_y[i]**2, mag_z[i]**2, mag_x[i]*mag_y[i], mag_x[i]*mag_z[i], mag_y[i]*mag_z[i], mag_x[i], mag_y[i], mag_z[i]) for i in range(data_size)])

# Least Squares
# LS = (D.T * D)^-1 * D.T * I(unit matrix)
a = np.linalg.pinv(D) @ np.ones(data_size)

if len(a) != 9:
    print(f'[ a Matrix size ERROR {len(a)} ]')
    exit()

print('[ A matrix ]')
print(a)

# create M(3 x 3) Positive Definite Matrix
# M = [ a1,   a4/2, a5/2,
#       a4/2, a2,   a6/2, 
#       a5/2, a6/2, a3 ]
M = np.array([[a[0], a[3]/2, a[4]/2],
              [a[3]/2, a[1], a[5]/2],
              [a[4]/2, a[5]/2, a[2]]])

print('[ M matrix ]')
print(M)

try:
    np.linalg.cholesky(M)
except np.linalg.LinAlgError:
    print('[ M matrix is not a Positive Definitr Matrix ]')
    exit()

# center = -(1/2)[a7, a8, a9]M^-1
center = -0.5 * (np.array([a[6], a[7], a[8]]) @ np.linalg.inv(M))
print('[ center ]')
print(center)

# scale factory
# SS = C * M * C.T + 1
Scale = (center @ M @ center.T + 1)/2500
print('[ SS ]')
print(Scale)

#  separate M matrix to Value and Vector matrix
values, vector = np.linalg.eig(M)
print('[ values ]')
print(values)
print('[ vector ]')
print(vector)

# get lambda
Scale_Matrix = np.diag(np.sqrt(Scale / np.abs(values)))

print('[ Scale Matrix ]')
print(Scale_Matrix)

# soft iron matrix
soft_iron_matrix = Scale_Matrix @ vector.T
print('[ soft iron matrix ]')
print(soft_iron_matrix)

# fix mag data
# mag_fix = soft_iron_matrix * (mag - center)
mag_org = np.array([(mag_x_org[i], mag_y_org[i], mag_z_org[i]) for i in range(data_size)])

# shift center
mag_shift = []
for i in range(data_size):
    # shift = soft_iron_matrix @ (mag_org[i] - center)
    shift = soft_iron_matrix @ mag_org[i]
    mag_shift.append(shift)

mag_shift_x = [d[0] for d in mag_shift]
mag_shift_y = [d[1] for d in mag_shift]
mag_shift_z = [d[2] for d in mag_shift]

# plot raw and fix data in same figure
create_plot(mag_x_org, mag_shift_x, 'Mag X Shift', 'Sample point', 'X', 'red')
create_plot(mag_y_org, mag_shift_y, 'Mag Y Shift', 'Sample point', 'Y', 'red')
create_plot(mag_z_org, mag_shift_z, 'Mag Z Shift', 'Sample point', 'Z', 'red')

fig = plt.figure(figsize=(6, 4))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
ax.set_zlim(-500, 500) 

ax.set_title('mag figure', fontsize=15)
ax.set_xlabel('X', fontsize=12)
ax.set_ylabel('Y', fontsize=12)
ax.set_zlabel('Z', fontsize=12)
scatter = ax.scatter(mag_shift_x, mag_shift_y, mag_shift_z, c='blue')
scatter = ax.scatter(mag_x_org, mag_y_org, mag_z_org, c='red')
plt.tight_layout()

plt.show()

# /Users/bit8/Desktop/develop/TDrone/TDrone_MCU_App/PY_Tool/mag_1.txt