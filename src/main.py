import time
import json
from typing import Dict, Union
from machine import I2C, Pin, SDCard
import lib.scd4x as scd4x
from ucollections import OrderedDict
import lib.sgp41 as sgp41
from lib.pms7003 import Pms7003
import lib.hdc302x as hdc302x

# Import CrowPanel for e-ink display
import lib.crowpanel as crowpanel

prev_dict: "OrderedDict[str, Union[float, None, int]]" = OrderedDict(
    {
        "CO2": None,
        "Temperature": None,
        "Humidity": None,
        "VOC Index": None,
        "NOx Index": None,
        "PM1.0": None,
        "PM2.5": None,
        "PM10": None,
    }
)

current_dict: "OrderedDict[str, Union[float, None, int]]" = OrderedDict(
    {
        "CO2": None,
        "Temperature": None,
        "Humidity": None,
        "VOC Index": None,
        "NOx Index": None,
        "PM1.0": None,
        "PM2.5": None,
        "PM10": None,
    }
)

unit_dict: Dict[str, str] = {
    "CO2": "ppm",
    "Temperature": "C",
    "Humidity": "%",
    "VOC Index": "idx",
    "NOx Index": "idx",
    "PM1.0": "ug/m3",
    "PM2.5": "ug/m3",
    "PM10": "ug/m3",
}


# Helper function to clear and update text at a position
def update_text(y_pos, text):
    """Clear text area and draw new text only if content changed"""
    # Clear the 20px high area at this position
    # First draw text, then clear, then redraw text, then show
    display.fill_rect_partial(0, y_pos, 400, 20, 1, update=False)  # Clear the rectangle
    display.text_partial(text, 0, y_pos, 0, update=False)  # Redraw text after clearing
    # display.show_partial(0, y_pos, 400, 20)  # Show only this region


def main():
    i2c = I2C(0, sda=Pin(8), scl=Pin(3), freq=100000)

    # Initialize SCD4X sensor
    try:
        scd = scd4x.SCD4X(i2c)
        print("Serial number:", [hex(i) for i in scd.serial_number])
    except Exception as e:
        print("Failed to initialize SCD4X:", e)
        scd = None

    # Initialize PMS7003 sensor
    # Configure UART2 with TX=14, RX=9
    try:
        pms = Pms7003(uart=2, tx=Pin(14), rx=Pin(9))
        print("PMS7003 initialized")
    except Exception as e:
        print("Failed to initialize PMS7003:", e)
        pms = None

    # Initialize SGP41
    try:
        sgp = sgp41.SGP41(i2c)
        # Execute conditioning (optional, but good for first run)
        # sgp.execute_conditioning()
        print("SGP41 initialized")
    except Exception as e:
        print("Failed to initialize SGP41:", e)
        sgp = None

    # Initialize HDC302x sensor
    try:
        hdc = hdc302x.HDC302x(i2c)
        print("HDC302x initialized")
    except Exception as e:
        print("Failed to initialize HDC302x:", e)
        hdc = None

    if scd:
        scd.start_periodic_measurement()
    print("Waiting for data...")

    # Clear display and show initial message
    # display.fill(1)
    display.clear()

    rotator = ["-", "\\", "|", "/"]
    rotator_index = 0
    while True:
        # display.clear()
        # Update display with latest CO2 data

        if pms and pms.data_ready:
            try:
                pm1_0 = pms.pm1_0_atmospheric
                pm2_5 = pms.pm2_5_atmospheric
                pm10_0 = pms.pm10_0_atmospheric

                prev_dict["PM1.0"] = current_dict["PM1.0"]
                current_dict["PM1.0"] = pm1_0
                prev_dict["PM2.5"] = current_dict["PM2.5"]
                current_dict["PM2.5"] = pm2_5
                prev_dict["PM10"] = current_dict["PM10"]
                current_dict["PM10"] = pm10_0

                print(
                    json.dumps(
                        {
                            "sensor": "pms7003",
                            "pm1_0": pm1_0,
                            "pm2_5": pm2_5,
                            "pm10_0": pm10_0,
                        }
                    )
                )
            except Exception as e:
                print("PMS7003 error:", e)

        if scd and scd.data_ready:
            co2_value = scd.CO2
            prev_dict["CO2"] = current_dict["CO2"]
            current_dict["CO2"] = co2_value

            print(
                json.dumps(
                    {
                        "sensor": "scd41",
                        "co2": co2_value,
                    }
                )
            )

        if hdc:
            hdc_temp = hdc.temperature
            hdc_hum = hdc.relative_humidity

            prev_dict["Temperature"] = current_dict["Temperature"]
            current_dict["Temperature"] = round(hdc_temp, 1)
            prev_dict["Humidity"] = current_dict["Humidity"]
            current_dict["Humidity"] = round(hdc_hum, 1)

            print(
                json.dumps(
                    {
                        "sensor": "hdc302x",
                        "temperature": hdc_temp,
                        "humidity": hdc_hum,
                    }
                )
            )

        if sgp:
            sraw_voc, sraw_nox = sgp.measure_raw(current_dict["Humidity"], current_dict["Temperature"])
            voc_index = sgp._voc_algo.process(sraw_voc)
            nox_index = sgp._nox_algo.process(sraw_nox)

            prev_dict["VOC Index"] = current_dict["VOC Index"]
            current_dict["VOC Index"] = voc_index
            prev_dict["NOx Index"] = current_dict["NOx Index"]
            current_dict["NOx Index"] = nox_index

            print(
                json.dumps(
                    {
                        "sensor": "sgp41",
                        "voc_raw": sraw_voc,
                        "nox_raw": sraw_nox,
                        "voc_index": voc_index,
                        "nox_index": nox_index,
                    }
                )
            )

        for i, (key, value) in enumerate(current_dict.items()):
            if value is not None:
                update_text((i + 1) * 20, f"{key}: {value} {unit_dict[key]}")
        rotator_index = 0 if rotator_index >= len(rotator) - 1 else rotator_index + 1
        update_text(0, rotator[rotator_index])

        # update partial all text (include all Y positions up to Y_NOX + 20)
        display.show_partial(0, 0, 400, (len(current_dict) + 1) * 20)

        time.sleep(1)


if __name__ == "__main__":
    # Initialize CrowPanel e-ink display
    panel = crowpanel.CrowPanel42()
    panel.led.on()
    display = panel.get_display()
    try:
        main()
    except Exception as e:
        print("Error:", e)
        update_text(0, str(e))
        display.show_partial(0, 0, 400, 20)
        time.sleep(10)
