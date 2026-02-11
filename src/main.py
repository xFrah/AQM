import time
import json
from machine import I2C, Pin, SDCard
import lib.scd4x as scd4x
import lib.sgp41 as sgp41
from lib.pms7003 import Pms7003

# Import CrowPanel for e-ink display
import lib.crowpanel as crowpanel


def main():
    i2c = I2C(0, sda=Pin(8), scl=Pin(3), freq=100000)

    # Initialize SCD4X sensor
    scd = scd4x.SCD4X(i2c)
    print("Serial number:", [hex(i) for i in scd.serial_number])

    # Initialize PMS7003 sensor (temporarily disabled)
    # Configure UART1 with TX=25, RX=26
    # pms = Pms7003(uart=2, tx=Pin(25), rx=Pin(26))
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

    scd.start_periodic_measurement()
    print("Waiting for data...")

    # Initialize CrowPanel e-ink display
    panel = crowpanel.CrowPanel42()
    panel.led.on()
    display = panel.get_display()
    
    # Clear display and show initial message
    # display.fill(1)
    display.clear()
    
    # Fixed Y positions for each data type
    Y_CO2 = 20
    Y_TEMP = 35
    Y_HUM = 50
    Y_VOC = 20
    Y_NOX = 35
    
    # Last known environment values for SGP41 compensation
    last_rh = None
    last_temp = None
    
    # Track previous values for change detection
    prev_co2 = None
    prev_temp = None
    prev_hum = None
    prev_voc = None
    prev_nox = None
    
    # Helper function to clear and update text at a position
    def update_text(y_pos, text):
        """Clear text area and draw new text only if content changed"""
        # Clear the 20px high area at this position
        display.fill_rect_partial(0, y_pos, 400, 20, 1)
        display.show_partial(0, y_pos, 400, 20)
        display.text_partial(text, 0, y_pos)

    while True:
        # Update display with latest CO2 data
        
        # PMS7003 temporarily disabled
        # if pms.data_ready:
        #     try:
        #         pm1_0 = pms.pm1_0_atmospheric
        #         pm2_5 = pms.pm2_5_atmospheric
        #         pm10_0 = pms.pm10_0_atmospheric
        #         display.text(f'PM1.0: {pm1_0} ug/m3', 0, y, 0)
        #         y += 15
        #         display.text(f'PM2.5: {pm2_5} ug/m3', 0, y, 0)
        #         y += 15
        #         display.text(f'PM10:  {pm10_0} ug/m3', 0, y, 0)
        #         y += 20
        #         print(
        #             json.dumps(
        #                 {
        #                     "sensor": "pms7003",
        #                     "pm1_0": pm1_0,
        #                     "pm2_5": pm2_5,
        #                     "pm10_0": pm10_0,
        #                 }
        #             )
        #         )
        #     except Exception as e:
        #         print("PMS7003 error:", e)
        #         display.text('PMS7003 error', 0, y, 0)
        #         y += 15

        if scd.data_ready:
            try:
                last_temp = scd.temperature
                last_rh = scd.relative_humidity
                co2_value = scd.CO2
                
                # Update CO2 only if changed
                if co2_value != prev_co2:
                    update_text(Y_CO2, f'CO2:   {co2_value} ppm')
                    prev_co2 = co2_value
                
                # Update Temp only if changed
                if last_temp != prev_temp:
                    update_text(Y_TEMP, f'Temp:  {last_temp:.1f} C')
                    prev_temp = last_temp
                
                # Update Hum only if changed
                if last_rh != prev_hum:
                    update_text(Y_HUM, f'Hum:   {last_rh:.1f} %')
                    prev_hum = last_rh
                print(
                    json.dumps(
                        {
                            "sensor": "scd41",
                            "co2": co2_value,
                            "temperature": last_temp,
                            "humidity": last_rh,
                        }
                    )
                )
            except Exception as e:
                print("SCD41 error:", e)
                display.text_partial('SCD41 error', 0, Y_CO2)

        if sgp:
            try:
                sraw_voc, sraw_nox = sgp.measure_raw(last_rh, last_temp)
                voc_index = sgp._voc_algo.process(sraw_voc)
                nox_index = sgp._nox_algo.process(sraw_nox)
                
                # Update VOC only if changed
                if voc_index != prev_voc:
                    update_text(Y_VOC, f'VOC:   {voc_index} idx')
                    prev_voc = voc_index
                
                # Update NOx only if changed
                if nox_index != prev_nox:
                    update_text(Y_NOX, f'NOx:   {nox_index} idx')
                    prev_nox = nox_index
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
            except Exception as e:
                print("SGP41 error:", e)
                display.text_partial('SGP41 error', 0, Y_VOC)
        else:
            # Clear VOC/NOx positions when sensor not available
            display.fill_rect_partial(0, Y_VOC, 400, 20, 1)
            display.show_partial(0, Y_VOC, 400, 20)
            display.fill_rect_partial(0, Y_NOX, 400, 20, 1)
            display.show_partial(0, Y_NOX, 400, 20)
            prev_voc = None
            prev_nox = None

        time.sleep(1)


if __name__ == "__main__":
    main()