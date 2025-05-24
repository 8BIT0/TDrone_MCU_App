import serial.tools.list_ports
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
    pass