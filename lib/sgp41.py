"""
SGP41 MicroPython Driver
Based on Sensirion I2C SGP41 Driver
"""

import time
import struct
from machine import I2C
from typing import Tuple, Optional
from sgp41_gas_index_algorithm import GasIndexAlgorithm

class SGP41:
    def __init__(self, i2c: I2C, address: int = 0x59, sampling_interval: float = 1.0) -> None:
        self._i2c = i2c
        self._addr = address
        self._voc_algo = GasIndexAlgorithm(GasIndexAlgorithm.ALGORITHM_TYPE_VOC, sampling_interval)
        self._nox_algo = GasIndexAlgorithm(GasIndexAlgorithm.ALGORITHM_TYPE_NOX, sampling_interval)

    def _crc(self, data: bytes) -> int:
        crc = 0xFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF

    def _write_command(self, command: int, data: Optional[bytes] = None) -> None:
        buf = struct.pack('>H', command)
        if data:
            for i in range(0, len(data), 2):
                chunk = data[i:i+2]
                buf += chunk
                buf += struct.pack('B', self._crc(chunk))
        self._i2c.writeto(self._addr, buf)

    def _read_result(self, length: int) -> bytes:
        # length is number of bytes to read (including CRC)
        data = self._i2c.readfrom(self._addr, length)
        result = bytearray()
        for i in range(0, length, 3):
            word = data[i:i+2]
            crc = data[i+2]
            if self._crc(word) != crc:
                raise RuntimeError("CRC Error")
            result.extend(word)
        return bytes(result)
        
    def execute_conditioning(self, relative_humidity: Optional[float] = None, temperature: Optional[float] = None) -> int:
        """
        This command starts the conditioning, i.e., the VOC pixel will be operated
        at the same temperature as it is by calling the measure_raw command
        while the NOx pixel will be operated at a different temperature for
        conditioning. This command returns only the measured raw signal of the VOC
        pixel SRAW_VOC.
        
        WARNING: To avoid damage to the sensing material the conditioning must not exceed 10s!
        
        Returns: int (raw VOC ticks)
        """
        if relative_humidity is None:
            rh_raw = 0x8000
        else:
            rh_raw = int(relative_humidity * 65535 / 100)
            
        if temperature is None:
            t_raw = 0x6666
        else:
            t_raw = int((temperature + 45) * 65535 / 175)
            
        # Command: 0x2612
        payload = struct.pack('>HH', rh_raw, t_raw)
        self._write_command(0x2612, payload)
        time.sleep(0.05)
        resp = self._read_result(3) # 2 bytes data + 1 CRC
        return struct.unpack('>H', resp)[0]

    def measure_raw(self, relative_humidity: Optional[float] = None, temperature: Optional[float] = None) -> Tuple[int, int]:
        """
        Read raw VOC signal and raw NOx signal.
        
        Returns: tuple(int, int) -> (voc_ticks, nox_ticks)
        """
        if relative_humidity is None:
            rh_raw = 0x8000
        else:
            rh_raw = int(relative_humidity * 65535 / 100)
            
        if temperature is None:
            t_raw = 0x6666
        else:
            t_raw = int((temperature + 45) * 65535 / 175)
            
        # Command: 0x2619
        payload = struct.pack('>HH', rh_raw, t_raw)
        self._write_command(0x2619, payload)
        time.sleep(0.05)
        resp = self._read_result(6) # 2 words * (2 bytes + 1 CRC)
        voc_ticks = struct.unpack('>H', resp[0:2])[0]
        nox_ticks = struct.unpack('>H', resp[2:4])[0]
        return voc_ticks, nox_ticks

    def execute_self_test(self) -> int:
        """
        Triggers the built-in self-test.
        Returns 0xD400 if all tests passed successfully.
        """
        # Command: 0x280E
        self._write_command(0x280E)
        time.sleep(0.32)
        resp = self._read_result(3)
        return struct.unpack('>H', resp)[0]

    def turn_heater_off(self) -> None:
        """
        Turns the hotplate off and stops the measurement.
        """
        # Command: 0x3615
        self._write_command(0x3615)
        time.sleep(0.001) # Post processing time
        
    def get_serial_number(self) -> int:
        """
        Returns the 48-bit serial number as an int.
        """
        # Command: 0x3682
        self._write_command(0x3682)
        time.sleep(0.001)
        resp = self._read_result(9) # 3 words * 3 bytes
        # resp has CRCs stripped by _read_result, so it is 6 bytes long
        words = struct.unpack('>HHH', resp)
        return (words[0] << 32) | (words[1] << 16) | words[2]
        
    def measure_index(self, relative_humidity: Optional[float] = None, temperature: Optional[float] = None) -> Tuple[int, int]:
        """
        Read raw signals and compute VOC and NOx gas index values.
        
        This should be called once per sampling interval (default 1s).
        The first ~45 seconds are a blackout period where both indexes return 0.
        After that, indexes range from 1 to 500.
        
        Args:
            relative_humidity: RH in %  (0-100), or None for default (50%)
            temperature:       Temp in °C, or None for default (25°C)
        
        Returns:
            tuple(int, int) -> (voc_index, nox_index)
        """
        sraw_voc, sraw_nox = self.measure_raw(relative_humidity, temperature)
        voc_index = self._voc_algo.process(sraw_voc)
        nox_index = self._nox_algo.process(sraw_nox)
        return voc_index, nox_index

    def get_voc_index(self, relative_humidity: Optional[float] = None, temperature: Optional[float] = None) -> int:
        """
        Read raw signals and return only the VOC index.
        
        Note: This also advances the NOx algorithm state internally.
        Call once per sampling interval.
        
        Returns:
            int: VOC index (0 during blackout, 1..500 afterwards)
        """
        voc_index, _ = self.measure_index(relative_humidity, temperature)
        return voc_index

    def get_nox_index(self, relative_humidity: Optional[float] = None, temperature: Optional[float] = None) -> int:
        """
        Read raw signals and return only the NOx index.
        
        Note: This also advances the VOC algorithm state internally.
        Call once per sampling interval.
        
        Returns:
            int: NOx index (0 during blackout, 1..500 afterwards)
        """
        _, nox_index = self.measure_index(relative_humidity, temperature)
        return nox_index

    def reset_index_algorithm(self) -> None:
        """
        Reset both VOC and NOx index algorithm states.
        Call this when resuming after a measurement interruption.
        Previously set tuning parameters are preserved.
        """
        self._voc_algo.reset()
        self._nox_algo.reset()

    def get_voc_states(self) -> Tuple[float, float]:
        """
        Get current VOC algorithm states for persistence.
        Can be used to skip the initial learning phase after a short interruption.
        NOTE: Only valid after at least 3 hours of continuous operation.
        
        Returns:
            tuple(float, float) -> (state0, state1)
        """
        return self._voc_algo.get_states()

    def set_voc_states(self, state0: float, state1: float) -> None:
        """
        Restore previously saved VOC algorithm states to skip initial learning phase.
        Do not use after interruptions longer than 10 minutes.
        Call this once after init/reset.
        
        Args:
            state0: Previously saved state0
            state1: Previously saved state1
        """
        self._voc_algo.set_states(state0, state1)

    # Alias methods
    def conditioning(self, relative_humidity: Optional[float] = None, temperature: Optional[float] = None) -> int:
        return self.execute_conditioning(relative_humidity, temperature)
        
    def measure_test(self) -> int:
        return self.execute_self_test()
