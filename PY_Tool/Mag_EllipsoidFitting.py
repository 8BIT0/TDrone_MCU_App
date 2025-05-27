import serial.tools.list_ports
import os
import time
from time import sleep
from pymavlink import mavutil
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

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

def Connect_SerialDev():
    avaliable_port = list(serial.tools.list_ports.comports())
    os.system('clear')
    print("[ detected port number ] -------- ", len(avaliable_port))

    FC_Found = False
    mav_connection = None

    # if your operation system is windows make sure you install AT32 and STM32 VCP driver already
    if len(avaliable_port):
        for port_info in avaliable_port:
            print(port_info)
            # if (port_info.description.find('H7FC') >= 0) or (port_info.manufacturer.find('8_B!T0') >= 0):
            if port_info.description.find('TDrone') >= 0:
                FC_Found = True
                print("\t[ Flight Controller Found ]")
                print("\t[ --- PORT INFO --- ]")
                print("\t[ name          ]: ", port_info.name)
                print("\t[ device        ]: ", port_info.device)
                print("\t[ pid           ]: ", port_info.pid)
                print("\t[ serial number ]: ", port_info.serial_number)
                print("\t[ description   ]: ", port_info.description)
                print("\t[ product       ]: ", port_info.product)
                print("\t[ manufacturer  ]: ", port_info.manufacturer)
                print("\r\n")
                break

        if not FC_Found:
            # manully check the flight controller is attach or not
            while True:
                input_code = input("select port: ")
                try:
                    input_code_i = int(input_code)
                    if int(input_code_i) >= avaliable_port.__len__():
                        print("input over range: {}".format(input_code_i))
                        continue
                    else:
                        print("selected port: {}".format(avaliable_port[input_code_i]))
                        port_info = avaliable_port[input_code_i]
                        FC_Found = True
                        break
                except ValueError:
                    print("Invalid input:{}".format(input_code))
                    continue

    # if found flight controller is attach
    # then open seleceted port
    if FC_Found:
        print("[ Flight Controller Port Open Successed ]\r\n")
        # communicate with the flight controller
        mav_connection = mavutil.mavlink_connection(port_info.device, baud = 460800, timeout = 5)
        print("[ Flight Controller is disconnected ]")
    else:
        print("\t[ Flight Controller Not Found ]")

    return mav_connection

def Mavlink_Parse(mav_connection, timeout = 60):
    if mav_connection is None:
        return

    x = np.array([0])
    y = np.array([0])
    z = np.array([0])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    start_time = time.time()
    print("mavlink parse start at {}".format(start_time))
    while time.time() - start_time < timeout:
        msg = mav_connection.recv_match(blocking = True, timeout = 1)
        if msg.get_type() == 'SCALED_IMU':
            for field in msg.get_fieldnames():
                if field == 'time_boot_ms':
                    time_stamp = getattr(msg, field)
                if field == 'xmag':
                    xmag = getattr(msg, field) * 0.1
                    x = np.append(x, xmag)
                if field == 'ymag':
                    ymag = getattr(msg, field) * 0.1
                    y = np.append(y, ymag)
                if field == 'zmag':
                    zmag = getattr(msg, field) * 0.1
                    z = np.append(z, zmag)
            print("time_stamp: {} xmag: {} ymag: {} zmag: {}".format(time_stamp, xmag, ymag, zmag))
        sleep(0.01)

    scatter = ax.scatter(x, y, z, c=z, cmap='viridis', s=100, alpha=0.8)
    ax.set_title('mag figure', fontsize=15)
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_zlabel('Z', fontsize=12)

    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_zlim(-500, 500) 
    
    cbar = plt.colorbar(scatter)
    cbar.set_label('数值', rotation=270, labelpad=20)

    plt.tight_layout()
    plt.show()

connect = Connect_SerialDev()
if connect is not None:
    Mavlink_Parse(connect)