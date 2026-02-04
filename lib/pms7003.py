from typing import Tuple, Union
import machine
import struct
import time


class UartError(Exception):
    pass


class Pms7003:

    START_BYTE_1 = 0x42
    START_BYTE_1_BYTES = bytes([START_BYTE_1])
    START_BYTE_2 = 0x4D
    START_BYTE_2_BYTES = bytes([START_BYTE_2])

    PMS_FRAME_LENGTH = 0
    PMS_PM1_0 = 1
    PMS_PM2_5 = 2
    PMS_PM10_0 = 3
    PMS_PM1_0_ATM = 4
    PMS_PM2_5_ATM = 5
    PMS_PM10_0_ATM = 6
    PMS_PCNT_0_3 = 7
    PMS_PCNT_0_5 = 8
    PMS_PCNT_1_0 = 9
    PMS_PCNT_2_5 = 10
    PMS_PCNT_5_0 = 11
    PMS_PCNT_10_0 = 12
    PMS_VERSION = 13
    PMS_ERROR = 14
    PMS_CHECKSUM = 15

    def __init__(self, uart, tx=None, rx=None):
        kwargs = {}
        if tx is not None:
            kwargs['tx'] = tx
        if rx is not None:
            kwargs['rx'] = rx
        self.uart = machine.UART(uart, baudrate=9600, bits=8, parity=None, stop=1, timeout=200, **kwargs)
        self._buffer = bytearray()

        self._pm1_0_standard: Union[int, None] = None
        self._pm2_5_standard: Union[int, None] = None
        self._pm10_0_standard: Union[int, None] = None
        self._pm1_0_atmospheric: Union[int, None] = None
        self._pm2_5_atmospheric: Union[int, None] = None
        self._pm10_0_atmospheric: Union[int, None] = None
        self._pcnt_0_3: Union[int, None] = None
        self._pcnt_0_5: Union[int, None] = None
        self._pcnt_1_0: Union[int, None] = None
        self._pcnt_2_5: Union[int, None] = None
        self._pcnt_5_0: Union[int, None] = None
        self._pcnt_10_0: Union[int, None] = None
        self._version: Union[int, None] = None
        self._error: Union[int, None] = None

    def __repr__(self):
        return "Pms7003({})".format(self.uart)

    @staticmethod
    def _assert_byte(byte, expected):
        if byte is None or len(byte) < 1 or ord(byte) != expected:
            return False
        return True

    @staticmethod
    def _format_bytearray(buffer):
        return "".join("0x{:02x} ".format(i) for i in buffer)

    def _send_cmd(self, request, response):

        nr_of_written_bytes = self.uart.write(request)

        if nr_of_written_bytes != len(request):
            raise UartError("Failed to write to UART")

        if response:
            time.sleep(2)
            buffer = self.uart.read(len(response))

            if buffer != response:
                raise UartError(
                    "Wrong UART response, expecting: {}, getting: {}".format(Pms7003._format_bytearray(response), Pms7003._format_bytearray(buffer))
                )

    @property
    def data_ready(self):
        # Read all available data into buffer
        while self.uart.any():
            data = self.uart.read()
            if data:
                self._buffer.extend(data)

        # Iterate backwards to find the latest valid frame
        # We start searching for START_BYTE_1 from the end
        limit = len(self._buffer)
        
        while True:
            # Find last occurrence of START_BYTE_1 before 'limit'
            start_idx = self._buffer.rfind(self.START_BYTE_1_BYTES, 0, limit)
            
            if start_idx == -1:
                # No start byte found
                break
                
            # Check if we have header 0x42 0x4D
            # Need at least 2 bytes (start_idx + 1 must exist)
            if start_idx + 1 >= len(self._buffer):
                # This 0x42 is at the very end. Keep it for next time.
                limit = start_idx
                continue

            if self._buffer[start_idx + 1] != self.START_BYTE_2:
                # Not a valid header
                limit = start_idx
                continue
                
            # Check if we have a full frame (32 bytes)
            if start_idx + 32 > len(self._buffer):
                # We have a header but not enough data for full frame yet.
                # This could be the start of the NEWEST packet arriving.
                # We should NOT discard it, but we can't process it yet.
                # Continue searching backwards for an *older* complete packet?
                # User wants "most recent". If the most recent is partial, 
                # we can't use it. We should look for the most recent *complete* packet.
                limit = start_idx
                continue
                
            # We have 32 bytes. Verify checksum.
            frame = self._buffer[start_idx : start_idx + 32]
            
            calculated_checksum = sum(frame[:30])
            expected_checksum = (frame[30] << 8) | frame[31]
            
            if calculated_checksum == expected_checksum:
                # Valid frame found!
                data = struct.unpack("!HHHHHHHHHHHHHBBH", frame[2:])
                self._update_data(data)
                
                # Keep only what follows this frame (partial newer data)
                self._buffer = self._buffer[start_idx + 32:]
                return True
            else:
                # Invalid checksum
                limit = start_idx
                continue
                
        # If buffer is getting too large with no valid data, trim it to prevent OOM
        # Keep last 64 bytes (enough for a split frame)
        if len(self._buffer) > 100:
             self._buffer = self._buffer[-64:]
             
        return False

    def _update_data(self, data: Tuple[int, ...]):
        self._pm1_0_standard = data[Pms7003.PMS_PM1_0]
        self._pm2_5_standard = data[Pms7003.PMS_PM2_5]
        self._pm10_0_standard = data[Pms7003.PMS_PM10_0]
        self._pm1_0_atmospheric = data[Pms7003.PMS_PM1_0_ATM]
        self._pm2_5_atmospheric = data[Pms7003.PMS_PM2_5_ATM]
        self._pm10_0_atmospheric = data[Pms7003.PMS_PM10_0_ATM]
        self._pcnt_0_3 = data[Pms7003.PMS_PCNT_0_3]
        self._pcnt_0_5 = data[Pms7003.PMS_PCNT_0_5]
        self._pcnt_1_0 = data[Pms7003.PMS_PCNT_1_0]
        self._pcnt_2_5 = data[Pms7003.PMS_PCNT_2_5]
        self._pcnt_5_0 = data[Pms7003.PMS_PCNT_5_0]
        self._pcnt_10_0 = data[Pms7003.PMS_PCNT_10_0]
        self._version = data[Pms7003.PMS_VERSION]
        self._error = data[Pms7003.PMS_ERROR]

    @property
    def pm1_0_standard(self):
        return self._pm1_0_standard

    @property
    def pm2_5_standard(self):
        return self._pm2_5_standard

    @property
    def pm10_0_standard(self):
        return self._pm10_0_standard

    @property
    def pm1_0_atmospheric(self):
        return self._pm1_0_atmospheric

    @property
    def pm2_5_atmospheric(self):
        return self._pm2_5_atmospheric

    @property
    def pm10_0_atmospheric(self):
        return self._pm10_0_atmospheric


class PassivePms7003(Pms7003):
    """
    More about passive mode here:
    https://github.com/teusH/MySense/blob/master/docs/pms7003.md
    https://patchwork.ozlabs.org/cover/1039261/
    https://joshefin.xyz/air-quality-with-raspberrypi-pms7003-and-java/
    """

    ENTER_PASSIVE_MODE_REQUEST = bytearray([Pms7003.START_BYTE_1, Pms7003.START_BYTE_2, 0xE1, 0x00, 0x00, 0x01, 0x70])
    ENTER_PASSIVE_MODE_RESPONSE = bytearray([Pms7003.START_BYTE_1, Pms7003.START_BYTE_2, 0x00, 0x04, 0xE1, 0x00, 0x01, 0x74])
    SLEEP_REQUEST = bytearray([Pms7003.START_BYTE_1, Pms7003.START_BYTE_2, 0xE4, 0x00, 0x00, 0x01, 0x73])
    SLEEP_RESPONSE = bytearray([Pms7003.START_BYTE_1, Pms7003.START_BYTE_2, 0x00, 0x04, 0xE4, 0x00, 0x01, 0x77])
    # NO response
    WAKEUP_REQUEST = bytearray([Pms7003.START_BYTE_1, Pms7003.START_BYTE_2, 0xE4, 0x00, 0x01, 0x01, 0x74])
    # data as response
    READ_IN_PASSIVE_REQUEST = bytearray([Pms7003.START_BYTE_1, Pms7003.START_BYTE_2, 0xE2, 0x00, 0x00, 0x01, 0x71])

    def __init__(self, uart):
        super().__init__(uart=uart)
        # use passive mode pms7003
        self._send_cmd(request=PassivePms7003.ENTER_PASSIVE_MODE_REQUEST, response=PassivePms7003.ENTER_PASSIVE_MODE_RESPONSE)

    def sleep(self):
        self._send_cmd(request=PassivePms7003.SLEEP_REQUEST, response=PassivePms7003.SLEEP_RESPONSE)

    def wakeup(self):
        self._send_cmd(request=PassivePms7003.WAKEUP_REQUEST, response=None)

    def request_measurement(self):
        self._send_cmd(request=PassivePms7003.READ_IN_PASSIVE_REQUEST, response=None)
