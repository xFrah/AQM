import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
from datetime import datetime, timedelta
import collections
import sys
import numpy as np

# Configuration
MAX_DURATION_MINUTES = 10
BAUD_RATE = 115200

# Data storage (deques for efficient pops from the left)
timestamps = collections.deque()
co2_data = collections.deque()
temp_data = collections.deque()
hum_data = collections.deque()

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
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('Real-time Air Quality Monitor')

# Styling
ax1.set_ylabel('CO2 (ppm)')
ax1.set_ylim(300, 1700) # Initial default
ax1.grid(True)

ax2.set_ylabel('Temperature (°C)')
ax2.set_ylim(-10, 40)
ax2.grid(True)

ax3.set_ylabel('Humidity (%)')
ax3.set_ylim(0, 100)
ax3.set_xlabel('Time')
ax3.grid(True)

# Date formatter for x-axis
date_fmt = mdates.DateFormatter('%H:%M:%S')
ax3.xaxis.set_major_formatter(date_fmt)

def update_data(frame):
    # Read all lines currently in the buffer
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            # Attempt to parse "CO2,temp,hum"
            parts = line.split(',')
            if len(parts) == 3:
                try:
                    co2 = float(parts[0])
                    temp = float(parts[1])
                    hum = float(parts[2])
                    
                    # Use current time for the data point
                    data_time = datetime.now()
                    
                    timestamps.append(data_time)
                    co2_data.append(co2)
                    temp_data.append(temp)
                    hum_data.append(hum)
                    
                    # Enforce 10-minute window removal based on data timestamps is optional 
                    # if we only show the window, but good to keep memory usage low.
                    # We can use the same MAX_DURATION_MINUTES.
                    cutoff = data_time - timedelta(minutes=MAX_DURATION_MINUTES)
                    while timestamps and timestamps[0] < cutoff:
                        timestamps.popleft()
                        co2_data.popleft()
                        temp_data.popleft()
                        hum_data.popleft()
                        
                except ValueError:
                    # Ignore lines that don't parse as floats (e.g. debug text)
                    pass
        except Exception as e:
            print(f"Error reading/parsing serial: {e}")

    # Current time for the plot window
    now = datetime.now()
    min_time = now - timedelta(minutes=MAX_DURATION_MINUTES)
    max_time = now

    # Update plots
    ax1.clear()
    ax2.clear()
    ax3.clear()
    
    # Re-apply styles after clear
    ax1.set_ylabel('CO2 (ppm)')
    # Dynamic Y-limit for CO2: at least 1700, but expand if data exceeds it
    current_max_co2 = max(co2_data) if co2_data else 0
    ax1.set_ylim(300, max(1700, current_max_co2 + 100)) # Add 100 buffer
    ax1.grid(True)
    if co2_data:
        ax1.set_title(f'CO2: {co2_data[-1]:.0f} ppm')
        
        # Calculate mean of the first 5% of data (oldest visible data)
        # This creates a baseline based on the left-most part of the graph
        n_points = len(co2_data)
        # Ensure we take at least 1 point, but only if we have data
        n_sample = max(1, int(n_points * 0.05))
        
        # Get the first n_sample items (oldest)
        # Since we cast to list for max() anyway, we can slice that or slice the deque
        # Note: list(deque) is O(N), slicing list is O(K). N is small (~600), so fine.
        subset = list(co2_data)[:n_sample]
        if subset:
            mean_val = sum(subset) / len(subset)
            ax1.axhline(y=mean_val, color='lightgrey', linestyle=':', linewidth=2)
        
        # Calculate slope (ppm/hour)
        if len(co2_data) > 1:
            # Convert datetimes to relative seconds
            t0 = timestamps[0]
            time_sec = np.array([(t - t0).total_seconds() for t in timestamps])
            values = np.array(co2_data)
            
            # Linear fit (degree 1) -> slope, intercept
            slope, _ = np.polyfit(time_sec, values, 1)
            
            # Convert slope (ppm/sec) to ppm/hour
            ppm_per_hour = slope * 3600
            
            sign = "+" if ppm_per_hour >= 0 else ""
            ax1.set_title(f'CO2: {co2_data[-1]:.0f} ppm ({sign}{ppm_per_hour:.0f} ppm/hour)')
        else:
            ax1.set_title(f'CO2: {co2_data[-1]:.0f} ppm')

    ax2.set_ylabel('Temperature (°C)')
    ax2.set_ylim(-10, 40)
    ax2.grid(True)
    if temp_data:
        ax2.set_title(f'Temperature: {temp_data[-1]:.1f} °C')
    
    ax3.set_ylabel('Humidity (%)')
    ax3.set_ylim(0, 100)
    ax3.set_xlabel('Time')
    ax3.grid(True)
    if hum_data:
        ax3.set_title(f'Humidity: {hum_data[-1]:.1f} %')
    
    # Plot data if available
    if timestamps:
        ax1.plot(timestamps, co2_data, 'r-', label='CO2')
        ax2.plot(timestamps, temp_data, 'g-', label='Temperature')
        ax3.plot(timestamps, hum_data, 'b-', label='Humidity')
    
    # Set fixed time window
    ax3.set_xlim(min_time, max_time)
    
    # Format x-axis
    ax3.xaxis.set_major_formatter(date_fmt)
    fig.autofmt_xdate()
    
    return []

# Animation
# Interval 1000ms = 1s. This determines how often the plot refreshes.
# It doesn't affect data reading frequency (which is done in the loop inside update_data)
ani = animation.FuncAnimation(fig, update_data, interval=1000, cache_frame_data=False)

print("Starting plotter... Close the window to stop.")
plt.show()

# Cleanup on close
ser.close()
