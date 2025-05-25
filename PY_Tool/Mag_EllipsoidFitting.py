import serial.tools.list_ports
from pymavlink import mavutil
from time import sleep
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
    research_cnt = research_cnt + 1
    os.system('clear')
    print("[ detected port number ] -------- ", len(avaliable_port))

    FC_Found = False
    
    # if your operation system is windows make sure you install AT32 and STM32 VCP driver already
    if len(avaliable_port):
        for port_info in avaliable_port:
            print(port_info)
            # if (port_info.description.find('H7FC') >= 0) or (port_info.manufacturer.find('8_B!T0') >= 0):
            if port_info.description.find('H7FC') >= 0:
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

    # if found flight controller is attach
    # then open seleceted port
    if FC_Found:
        FC_port = serial.Serial(port_info.device, 460800, 5)
        if FC_port.is_open:
            print("[ Flight Controller Port Open Successed ]\r\n")
            # communicate with the flight controller
            sleep(0.2)

            FC_port.close()
            print("[ Flight Controller is disconnected ]")
        else :
            print("\t[ Flight Controller Port Open Failed ]")
    else :
        print("\t[ Flight Controller Not Found ]")
        print("\t[ Research ] ----------------- ", research_cnt)
        sleep(1)