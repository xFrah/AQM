import time
from machine import I2C, Pin

def main():
    # Using the pins you confirmed worked for SGP41
    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)
    
    print("Scanning I2C bus...")
    devices = i2c.scan()
    
    if devices:
        print("Devices found:", [hex(device) for device in devices])
    else:
        print("No I2C devices found")

if __name__ == "__main__":
    main()
