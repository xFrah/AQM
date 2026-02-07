import time
import json
from machine import I2C, Pin
import lib.scd4x as scd4x
import lib.bno085 as bno085
import lib.sgp41 as sgp41
from lib.pms7003 import Pms7003


def main():
    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)

    # Initialize SCD4X sensor
    scd = scd4x.SCD4X(i2c)
    print("Serial number:", [hex(i) for i in scd.serial_number])

    # Initialize PMS7003 sensor
    # Configure UART1 with TX=25, RX=26
    pms = Pms7003(uart=2, tx=Pin(25), rx=Pin(26))

    # Initialize BNO085
    try:
        bno = bno085.BNO085(i2c)
        bno.enable_feature(bno085.BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(bno085.BNO_REPORT_GYROSCOPE)
        bno.enable_feature(bno085.BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(bno085.BNO_REPORT_ROTATION_VECTOR)
        print("BNO085 initialized")
    except Exception as e:
        print("Failed to initialize BNO085:", e)
        bno = None

    # Initialize SGP41
    try:
        sgp = sgp41.SGP41(i2c)
        # Execute conditioning (optional, but good for first run)
        # sgp.execute_conditioning()
        print("SGP41 initialized")
    except Exception as e:
        print("Failed to initialize SGP41:", e)
        sgp = None

    scd.start_periodic_measurement()
    print("Waiting for data...")

    while True:
        if pms.data_ready:
            try:
                print(
                    json.dumps(
                        {
                            "sensor": "pms7003",
                            "pm1_0": pms.pm1_0_atmospheric,
                            "pm2_5": pms.pm2_5_atmospheric,
                            "pm10_0": pms.pm10_0_atmospheric,
                        }
                    )
                )
            except Exception as e:
                print("PMS7003 error:", e)

        if scd.data_ready:
            try:
                print(
                    json.dumps(
                        {
                            "sensor": "scd41",
                            "co2": scd.CO2,
                            "temperature": scd.temperature,
                            "humidity": scd.relative_humidity,
                        }
                    )
                )
            except Exception as e:
                print("SCD41 error:", e)

        if sgp:
            try:
                voc, nox = sgp.measure_raw()
                print(
                    json.dumps(
                        {
                            "sensor": "sgp41",
                            "voc_raw": voc,
                            "nox_raw": nox,
                        }
                    )
                )
            except Exception as e:
                print("SGP41 error:", e)

        if bno:
            try:
                accel = bno.acceleration
                gyro = bno.gyro
                mag = bno.magnetic
                quat = bno.quaternion
                print(
                    json.dumps(
                        {
                            "sensor": "bno085",
                            "accel": accel,
                            "gyro": gyro,
                            "mag": mag,
                            "quat": quat,
                        }
                    )
                )
            except Exception as e:
                print("BNO085 error:", e)

        time.sleep(1)


if __name__ == "__main__":
    main()
