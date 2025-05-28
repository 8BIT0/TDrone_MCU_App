import serial.tools.list_ports
import os
import time
from time import sleep
from pymavlink import mavutil
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import math
import tkinter as tk
import threading
import queue

scatter = None
dsp_mag_x, dsp_mag_y, dsp_mag_z, color = [], [], [], []
stop_event = threading.Event()
mav_connection = None

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
        if type(mav_connection) != None:
            print("[ Flight Controller is connected ]")
        else:
            print("[ Flight Controller is not connected ]")
    else:
        print("\t[ Flight Controller Not Found ]")

def Canvas_Init():
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_zlim(-500, 500) 

    ax.set_title('mag figure', fontsize=15)
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_zlabel('Z', fontsize=12)
    return (scatter,)

def Canvas_Update(frame, queue, scatter):
    new_points = []
    while not queue.empty():
        try:
            new_points.append(queue.get_nowait())
        except queue.Empty:
            sleep(0.01)
            continue

    if new_points:
        for point in new_points:
            timestamp, x, y, z = point
            dsp_mag_x.append(x)
            dsp_mag_y.append(y)
            dsp_mag_z.append(z)

        scatter._offsets3d = (dsp_mag_x, dsp_mag_y, dsp_mag_z)

    return (scatter,)

def Mavlink_Parse(queue):
    if mav_connection is None:
        return

    while True:
        msg = mav_connection.recv_match(blocking = True, timeout = 0.1)
        if type(msg) != None and msg.get_type() == 'SCALED_IMU':
            time_stamp = getattr(msg, 'time_boot_ms')
            xmag = round(getattr(msg, 'xmag') * 0.1, 2)
            ymag = round(getattr(msg, 'ymag') * 0.1, 2)
            zmag = round(getattr(msg, 'zmag') * 0.1, 2)
            mag_data = [time_stamp, xmag, ymag, zmag]
            print(mag_data)
            queue.put(mag_data)

def on_close(event):
    print("quit...")
    stop_event.set()
    if type(mav_connection) != None:
        mav_connection.close()
    plt.close()

Connect_SerialDev()
if mav_connection is not None:
    # create file
    # script_path = os.path.abspath(__file__)
    # cnv_file_name = os.path.dirname(script_path) + os.sep
    # cnv_file_name = cnv_file_name + input('convert file name: ') + '.txt'
    # with open(cnv_file_name, 'w') as cnv_file:
    #     cnv_file.write(str_cnv)
    #     cnv_file.close()

    mag_queue = queue.Queue()
    fig = plt.figure(figsize=(6, 4))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter([], [], [], c='blue')
    
    fig.canvas.mpl_connect('close_event', on_close)

    # create canvas for plot figure thread
    serial_thread = threading.Thread(target = Mavlink_Parse, args = (mag_queue))
    serial_thread.start()

    ani = animation.FuncAnimation(fig,
                                  Canvas_Update, 
                                  frames = None, 
                                  init_func = Canvas_Init,
                                  fargs = (mag_queue, scatter),
                                  blit = False, interval = 10, 
                                  cache_frame_data = False)
    plt.tight_layout()
    plt.show()

