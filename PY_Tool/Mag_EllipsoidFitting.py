import serial.tools.list_ports
import os
import time
from pymavlink import mavutil
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

def Mavlink_Parse(mav_connection, timeout = 10):
    if mav_connection is None:
        return
    
    start_time = time.time()
    print("mavlink parse start at {}".format(start_time))
    while time.time() - start_time < timeout:
        msg = mav_connection.recv_match(blocking = True, timeout = 1)
        if msg.get_type() == 'SCALED_IMU':
            for field in msg.get_fieldnames():
                print(f"  {field}: {getattr(msg, field)}")


connect = Connect_SerialDev()
if connect is not None:
    Mavlink_Parse(connect)