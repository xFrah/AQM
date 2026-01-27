import time
from machine import I2C, Pin
import lib.scd4x as scd4x

def main():
    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)
    
    # Initialize SCD4X sensor
    scd = scd4x.SCD4X(i2c)
    print("Serial number:", [hex(i) for i in scd.serial_number])
    
    scd.start_periodic_measurement()
    print("Waiting for data...")

    while True:
        if scd.data_ready:
            print(",".join([str(scd.CO2), str(scd.temperature), str(scd.relative_humidity)]))
        
        time.sleep(1)

if __name__ == "__main__":
    main()
