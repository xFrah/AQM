import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
from datetime import datetime, timedelta
import collections
import sys
import numpy as np
import json
import pandas as pd
import math

# Configuration
MAX_DURATION_MINUTES = 10
BAUD_RATE = 115200
MOVING_AVERAGE_WINDOW = 30  # Number of points for moving average

# Data storage (deques for efficient pops from the left)
timestamps = collections.deque()
co2_data = collections.deque()
temp_data = collections.deque()

pms_timestamps = collections.deque()
pm1_data = collections.deque()
pm25_data = collections.deque()
pm10_data = collections.deque()

sgp_timestamps = collections.deque()
voc_data = collections.deque()
nox_data = collections.deque()

bno_timestamps = collections.deque()
accel_data = collections.deque()

def calculate_moving_average(data, window_size):
    """Calculates the simple moving average."""
    if len(data) < window_size:
        return list(data)
    
    # Use pandas rolling window for efficient calculation
    return pd.Series(list(data)).rolling(window=window_size, min_periods=1).mean().tolist()

def find_serial_port():
    """Attempts to auto-detect the ESP32 serial port."""
    ports = list(serial.tools.list_ports.comports())
    # Prioritize ports that look like ESP32/USB-Serial
    for p in ports:
        desc = p.description.lower()
        if "cp210" in desc or "ch340" in desc or "usb serial" in desc or "jtag" in desc:
            return p.device
    
    # Fallback: return the first available port
    if ports:
        return ports[0].device
    return None

def init_serial():
    port = find_serial_port()
    if not port:
        print("Error: No serial port found. Please connect your ESP32.")
        sys.exit(1)
    
    print(f"Connecting to {port} at {BAUD_RATE} baud...")
    try:
        return serial.Serial(port, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

# Initialize Serial
ser = init_serial()

# Setup Plot
# 3 rows, 2 columns
fig, ((ax1, ax4), (ax2, ax5), (ax3, ax6)) = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
fig.suptitle('Real-time Air Quality Monitor')

# Styling - Column 1
ax1.set_ylabel('CO2 (ppm)')
ax1.set_ylim(300, 1700) # Initial default
ax1.grid(True)

ax2.set_ylabel('Temperature (°C)')
ax2.set_ylim(-10, 40)
ax2.grid(True)

ax3.set_ylabel('Humidity (%)') # Label kept for consistency, but plotting PM
ax3.set_ylim(0, 100)
ax3.set_xlabel('Time')
ax3.grid(True)

# Styling - Column 2
ax4.set_ylabel('VOC Index')
ax4.set_ylim(0, 500)
ax4.grid(True)

ax5.set_ylabel('NOx Index')
ax5.set_ylim(0, 500)
ax5.grid(True)

ax6.set_ylabel('Accel Intensity (m/s²)')
ax6.set_xlabel('Time')
ax6.grid(True)


# Date formatter for x-axis
date_fmt = mdates.DateFormatter('%H:%M:%S')
ax3.xaxis.set_major_formatter(date_fmt)
ax6.xaxis.set_major_formatter(date_fmt)

def update_data(frame):
    # Read all lines currently in the buffer
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            try:
                data = json.loads(line)
                sensor_type = data.get("sensor")
                data_time = datetime.now()
                cutoff = data_time - timedelta(minutes=MAX_DURATION_MINUTES)

                if sensor_type == "scd41":
                    co2 = float(data.get("co2", 0))
                    temp = float(data.get("temperature", 0))
                    
                    timestamps.append(data_time)
                    co2_data.append(co2)
                    temp_data.append(temp)
                    
                    while timestamps and timestamps[0] < cutoff:
                        timestamps.popleft()
                        co2_data.popleft()
                        temp_data.popleft()
                        
                elif sensor_type == "pms7003":
                    pm1 = float(data.get('pm1_0', 0))
                    pm25 = float(data.get('pm2_5', 0))
                    pm10 = float(data.get('pm10_0', 0))
                    
                    pms_timestamps.append(data_time)
                    pm1_data.append(pm1)
                    pm25_data.append(pm25)
                    pm10_data.append(pm10)
                    
                    while pms_timestamps and pms_timestamps[0] < cutoff:
                        pms_timestamps.popleft()
                        pm1_data.popleft()
                        pm25_data.popleft()
                        pm10_data.popleft()

                elif sensor_type == "sgp41":
                    voc = float(data.get('voc_index', 0))
                    nox = float(data.get('nox_index', 0))

                    sgp_timestamps.append(data_time)
                    voc_data.append(voc)
                    nox_data.append(nox)

                    while sgp_timestamps and sgp_timestamps[0] < cutoff:
                        sgp_timestamps.popleft()
                        voc_data.popleft()
                        nox_data.popleft()

                elif sensor_type == "bno085":
                    accel = data.get('accel')
                    if accel and len(accel) == 3:
                        x, y, z = accel
                        intensity = math.sqrt(x*x + y*y + z*z)
                        
                        bno_timestamps.append(data_time)
                        accel_data.append(intensity)

                        while bno_timestamps and bno_timestamps[0] < cutoff:
                            bno_timestamps.popleft()
                            accel_data.popleft()

            except json.JSONDecodeError:
                # Fallback or ignore non-JSON lines
                pass
        except Exception as e:
            print(f"Error reading/parsing serial: {e}")

    # Current time for the plot window
    now = datetime.now()
    min_time = now - timedelta(minutes=MAX_DURATION_MINUTES)
    max_time = now

    # Update plots
    for ax in [ax1, ax2, ax3, ax4, ax5, ax6]:
        ax.clear()
        ax.grid(True)
    
    # Re-apply styles after clear
    ax1.set_ylabel('CO2 (ppm)')
    current_max_co2 = max(co2_data) if co2_data else 0
    ax1.set_ylim(300, max(1700, current_max_co2 + 100))
    if co2_data:
        ax1.set_title(f'CO2: {co2_data[-1]:.0f} ppm')

    ax2.set_ylabel('Temperature (°C)')
    ax2.set_ylim(-10, 40)
    if temp_data:
        ax2.set_title(f'Temperature: {temp_data[-1]:.1f} °C')
    
    ax3.set_ylabel('PM (µg/m³)')
    current_max_pm = max(max(pm1_data) if pm1_data else 0, max(pm25_data) if pm25_data else 0, max(pm10_data) if pm10_data else 0)
    ax3.set_ylim(0, current_max_pm * 1.3 if current_max_pm > 10 else 10)
    ax3.set_xlabel('Time')
    if pm25_data:
        ax3.set_title(f'PM1.0: {pm1_data[-1]}, PM2.5: {pm25_data[-1]}, PM10: {pm10_data[-1]}')

    ax4.set_ylabel('VOC Index')
    ax4.set_ylim(0, 500)
    if voc_data:
        ax4.set_title(f'VOC Index: {voc_data[-1]:.0f}')
        
    ax5.set_ylabel('NOx Index')
    ax5.set_ylim(0, 500)
    if nox_data:
        ax5.set_title(f'NOx Index: {nox_data[-1]:.0f}')
        
    ax6.set_ylabel('Accel Intensity (m/s²)')
    ax6.set_xlabel('Time')
    if accel_data:
        ax6.set_title(f'Accel: {accel_data[-1]:.2f} m/s²')

    # Plot data if available
    if timestamps:
        co2_ma = calculate_moving_average(co2_data, MOVING_AVERAGE_WINDOW)
        temp_ma = calculate_moving_average(temp_data, MOVING_AVERAGE_WINDOW)
        
        ax1.plot(timestamps, co2_ma, 'r-', label=f'CO2 (MA{MOVING_AVERAGE_WINDOW})')
        ax2.plot(timestamps, temp_ma, 'g-', label=f'Temp (MA{MOVING_AVERAGE_WINDOW})')
    
    if pms_timestamps:
        pm1_ma = calculate_moving_average(pm1_data, MOVING_AVERAGE_WINDOW)
        pm25_ma = calculate_moving_average(pm25_data, MOVING_AVERAGE_WINDOW)
        pm10_ma = calculate_moving_average(pm10_data, MOVING_AVERAGE_WINDOW)

        ax3.plot(pms_timestamps, pm1_ma, 'b-', label=f'PM1.0')
        ax3.plot(pms_timestamps, pm25_ma, 'y-', label=f'PM2.5')
        ax3.plot(pms_timestamps, pm10_ma, 'm-', label=f'PM10')
        ax3.legend(loc="upper left", fontsize="small")

    if sgp_timestamps:
        voc_ma = calculate_moving_average(voc_data, MOVING_AVERAGE_WINDOW)
        nox_ma = calculate_moving_average(nox_data, MOVING_AVERAGE_WINDOW)
        
        ax4.plot(sgp_timestamps, voc_ma, 'c-', label='VOC')
        ax5.plot(sgp_timestamps, nox_ma, 'k-', label='NOx')

    if bno_timestamps:
        accel_ma = calculate_moving_average(accel_data, 5) # Smaller window for accel
        ax6.plot(bno_timestamps, accel_ma, 'purple', label='Intensity')

    # Set fixed time window
    ax3.set_xlim(min_time, max_time)
    ax6.set_xlim(min_time, max_time)
    
    # Format x-axis
    ax3.xaxis.set_major_formatter(date_fmt)
    ax6.xaxis.set_major_formatter(date_fmt)
    fig.autofmt_xdate()
    
    return []

# Animation
ani = animation.FuncAnimation(fig, update_data, interval=1000, cache_frame_data=False)

print("Starting plotter... Close the window to stop.")
plt.show()

# Cleanup on close
ser.close()
