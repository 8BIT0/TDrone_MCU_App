import os
from time import sleep
from enum import Enum
import math
import matplotlib.pyplot as plt
import numpy as np

class MagData_Index(Enum):
    Mag_Time        = 0
    Mag_X           = 1
    Mag_Y           = 2
    Mag_Z           = 3

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
    new_data = []
    for d in data:
        if d >= min_val and d <= max_val:
            new_data.append(d)

    return new_data

def Load_DataFile() -> list:
    file_path = input('[ Mag data file path ]:')
    mag_data_list = []
    try:
        with open(file_path, 'r') as data:
            for line in data:
                mag_data_list.append(line.split(' '))
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

time_stamp_list, mag_x_org, mag_y_org, mag_z_org = mag_data
diff_arr = np.diff(np.array(time_stamp_list)).tolist()

# show mag data time stamp diff
time_diff_fig = plt.figure(figsize=(8, 6))
plt.plot(diff_arr, color='blue', marker='o', linestyle='-')
plt.title('time diff', fontsize=16)
plt.xlabel('sample point', fontsize=12)
plt.ylabel('ms diff', fontsize=12)
plt.tight_layout()
plt.show()

# 3 sigma check
# noticed doing 3 sigma check may occur some bug
mag_x = np.array(ThreeSigma_Check(mag_x_org))
mag_y = np.array(ThreeSigma_Check(mag_y_org))
mag_z = np.array(ThreeSigma_Check(mag_z_org))

mag_x_mean = np.mean(mag_x)
mag_y_mean = np.mean(mag_y)
mag_z_mean = np.mean(mag_z)

if mag_x_mean == np.NaN or mag_y_mean == np.NaN or mag_z_mean == np.NaN:
    print('[ mag data mean is NaN ]')
    exit()

mag_x_elli = mag_x - mag_x_mean
mag_y_elli = mag_y - mag_y_mean
mag_z_elli = mag_z - mag_z_mean

# a1x^2 + a2y^2 + a3z^2 + a4xy + a5xz + a6yz + a7x + a8y + a9z = 1
# x^2, y^2, z^2, xy, xz, yz, x, y, z


# Least Squares
# LS = (Mag_Mea.T * Mag_Mea)^-1 * Mag_Mea.T * I(unit matrix)