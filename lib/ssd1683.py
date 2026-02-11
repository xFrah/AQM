"""
Enhanced SSD1683 E-Paper Driver for CrowPanel ESP32 4.2"
Supports full, fast, and partial updates based on vendor specifications
"""

from utime import sleep_ms
from machine import SPI, Pin
from framebuf import FrameBuffer, MONO_HLSB


class SSD1683(FrameBuffer):
    """
    Driver for SSD1683-based 4.2" E-Paper Display (400x300)

    Features:
    - Full refresh (slow, clean, removes ghosting)
    - Fast refresh (faster full-screen update)
    - Partial refresh (update only a region)
    - Dual RAM management for differential updates
    """

    def __init__(self, width=400, height=300, cs=45, dc=46, rst=47, busy=48, 
                 spi_id=1, sck=12, mosi=11):
        """
        Initialize the SSD1683 driver

        Args:
            width: Display width in pixels (default 400)
            height: Display height in pixels (default 300)
            cs: Chip select pin
            dc: Data/command pin
            rst: Reset pin
            busy: Busy status pin
            spi_id: SPI bus ID
            sck: SPI clock pin
            mosi: SPI MOSI pin
        """
        self._w = width
        self._h = height
        self._buf = bytearray(width * height // 8)
        super().__init__(self._buf, width, height, MONO_HLSB)

        # GPIO setup
        self._cs = Pin(cs, Pin.OUT)
        self._dc = Pin(dc, Pin.OUT)
        self._rst = Pin(rst, Pin.OUT)
        self._busy = Pin(busy, Pin.IN)

        # SPI setup
        self._spi = SPI(spi_id,
                        baudrate=4_000_000,
                        sck=Pin(sck),
                        mosi=Pin(mosi),
                        firstbit=SPI.MSB)

        # Initialize display
        self.init()

    # ============ Low-level SPI Communication ============

    def _cmd(self, b):
        """Send a command byte"""
        self._cs(0)
        self._dc(0)
        self._spi.write(bytearray([b]))
        self._cs(1)

    def _dat(self, b):
        """Send a data byte"""
        self._cs(0)
        self._dc(1)
        self._spi.write(bytearray([b]))
        self._cs(1)

    def _data_bulk(self, data):
        """Send bulk data (bytes or bytearray)"""
        self._cs(0)
        self._dc(1)
        self._spi.write(data)
        self._cs(1)

    def _wait(self):
        """Wait for the display to be ready (BUSY pin low)"""
        timeout = 0
        while self._busy.value() == 1:
            sleep_ms(10)
            timeout += 1
            if timeout > 500:  # 5 second timeout
                print("Warning: BUSY timeout")
                break

    def _reset(self):
        """Hardware reset sequence"""
        self._rst.value(1)
        sleep_ms(100)
        self._rst.value(0)
        sleep_ms(10)
        self._rst.value(1)
        sleep_ms(10)

    # ============ Display Configuration ============

    def _pos(self, x1, y1, x2, y2):
        """
        Set RAM address window

        Args:
            x1, y1: Start position (top-left)
            x2, y2: End position (bottom-right)
        """
        # X address is in units of 8 pixels
        self._cmd(0x44)  # SET_RAM_X_ADDRESS_START_END_POSITION
        self._dat((x1 >> 3) & 0xFF)
        self._dat((x2 >> 3) & 0xFF)

        # Y address is in pixels
        self._cmd(0x45)  # SET_RAM_Y_ADDRESS_START_END_POSITION
        self._dat(y1 & 0xFF)
        self._dat((y1 >> 8) & 0xFF)
        self._dat(y2 & 0xFF)
        self._dat((y2 >> 8) & 0xFF)

    def _cur(self, x, y):
        """
        Set RAM address cursor/counter

        Args:
            x: X position (in units of 8 pixels)
            y: Y position (in pixels)
        """
        self._cmd(0x4E)  # SET_RAM_X_ADDRESS_COUNTER
        self._dat((x >> 3) & 0xFF)

        self._cmd(0x4F)  # SET_RAM_Y_ADDRESS_COUNTER
        self._dat(y & 0xFF)
        self._dat((y >> 8) & 0xFF)

    # ============ Update Modes ============

    def _update_full(self):
        """
        Full update with complete waveform
        - Slowest (~2-3 seconds)
        - Removes ghosting
        - Use after several partial updates
        """
        self._cmd(0x22)  # DISPLAY_UPDATE_CONTROL_2
        self._dat(0xF7)  # Full update sequence
        self._cmd(0x20)  # MASTER_ACTIVATION
        self._wait()

    def _update_fast(self):
        """
        Fast full-screen update
        - Faster than full (~1.5 seconds)
        - May accumulate slight ghosting
        - Good for frequent full-screen changes
        """
        self._cmd(0x22)  # DISPLAY_UPDATE_CONTROL_2
        self._dat(0xC7)  # Fast update sequence
        self._cmd(0x20)  # MASTER_ACTIVATION
        self._wait()

    def _update_part(self):
        """
        Partial update waveform
        - Fastest (~0.3 seconds)
        - Updates only changed pixels in window
        - Ghosting accumulates - do full update every 5-10 partials
        """
        self._cmd(0x22)  # DISPLAY_UPDATE_CONTROL_2
        self._dat(0xFF)  # Partial update sequence (for SSD1683 4.2")
        self._cmd(0x20)  # MASTER_ACTIVATION
        self._wait()

    # ============ Initialization ============

    def init(self):
        """
        Initialize display with default (full refresh) settings
        """
        self._reset()
        self._wait()

        self._cmd(0x12)  # SOFT_RESET
        self._wait()

        # Display update control
        self._cmd(0x21)  # DISPLAY_UPDATE_CONTROL_1
        self._dat(0x40)  # Enable clock signal, Enable analog
        self._dat(0x00)

        # Border waveform
        self._cmd(0x3C)  # BORDER_WAVEFORM_CONTROL
        self._dat(0x05)  # Follow LUT

        # Data entry mode: X increment, Y increment
        self._cmd(0x11)  # DATA_ENTRY_MODE_SETTING
        self._dat(0x03)  # X+, Y+ mode

        # Set full window
        self._pos(0, 0, self._w - 1, self._h - 1)
        self._cur(0, 0)
        self._wait()

    def init_fast(self, mode_1s=False):
        """
        Initialize display for fast/partial refresh mode
        Loads temperature-compensated LUTs for faster updates

        Args:
            mode_1s: If True, use 1s profile; if False, use 1.5s profile
        """
        self._reset()
        self._wait()

        self._cmd(0x12)  # SOFT_RESET
        self._wait()

        # Display update control
        self._cmd(0x21)
        self._dat(0x40)
        self._dat(0x00)

        # Border waveform
        self._cmd(0x3C)
        self._dat(0x05)

        # Write temperature register (speed profile)
        self._cmd(0x1A)  # WRITE_TEMPERATURE_REGISTER
        self._dat(0x5A if mode_1s else 0x6E)  # 1s vs 1.5s profile

        # Load temperature value into LUT
        self._cmd(0x22)
        self._dat(0x91)  # Load temperature value
        self._cmd(0x20)
        self._wait()

        # Data entry mode
        self._cmd(0x11)
        self._dat(0x03)

        # Set full window
        self._pos(0, 0, self._w - 1, self._h - 1)
        self._cur(0, 0)
        self._wait()

    # ============ Display Methods ============

    def clear(self, color=1):
        """
        Clear the display and synchronize both internal RAMs
        This is important before using partial updates

        Args:
            color: 1 for white (default), 0 for black
        """
        byte_val = 0xFF if color else 0x00
        size = len(self._buf)

        # Create buffer
        clear_buf = bytes([byte_val] * size)

        # Write to NEW data RAM (0x24)
        self._cmd(0x24)  # WRITE_RAM_BW
        self._data_bulk(clear_buf)

        # Write to OLD data RAM (0x26) - important for differential updates
        self._cmd(0x26)  # WRITE_RAM_RED (used as previous frame buffer)
        self._data_bulk(clear_buf)

        # Full update to display
        self._update_full()

        # Sync our framebuffer
        for i in range(size):
            self._buf[i] = byte_val

    def show(self):
        """
        Display the current framebuffer content (full screen, full update)
        This is the slowest but cleanest update - removes all ghosting
        """
        self._cmd(0x24)  # WRITE_RAM_BW
        self._data_bulk(self._buf)
        self._update_full()

    def show_fast(self):
        """
        Display the current framebuffer content with fast update
        Faster than show() but may accumulate ghosting over time
        """
        self._cmd(0x24)  # WRITE_RAM_BW
        self._data_bulk(self._buf)
        self._update_fast()

    def show_partial(self, x, y, w, h):
        """
        Update only a rectangular region of the display
        This is the fastest update mode (~0.3s)

        IMPORTANT: 
        - Call clear() first to sync internal RAMs
        - x and w should be multiples of 8 for alignment
        - Do a full update every 5-10 partial updates to clear ghosting

        Args:
            x: X position (left edge, should be multiple of 8)
            y: Y position (top edge)
            w: Width in pixels (should be multiple of 8)
            h: Height in pixels
        """
        # Validate alignment
        if (x % 8) != 0 or (w % 8) != 0:
            raise ValueError("x and w must be multiples of 8 for show_partial()")

        # Validate bounds
        if x < 0 or y < 0 or x + w > self._w or y + h > self._h:
            raise ValueError("Partial window out of display bounds")

        # Configure for partial update (from vendor driver)
        self._cmd(0x3C)  # BORDER_WAVEFORM_CONTROL
        self._dat(0x80)  # Disable border output during partial

        self._cmd(0x21)  # DISPLAY_UPDATE_CONTROL_1
        self._dat(0x00)
        self._dat(0x00)

        self._cmd(0x3C)  # BORDER_WAVEFORM_CONTROL
        self._dat(0x80)

        # Set data entry mode
        self._cmd(0x11)  # DATA_ENTRY_MODE_SETTING
        self._dat(0x03)  # X+, Y+ mode

        # Set address window and cursor for the region
        self._pos(x, y, x + w - 1, y + h - 1)
        self._cur(x, y)

        # Stream only the bytes covering this region
        bytes_per_row = self._w // 8  # 400 / 8 = 50
        window_bytes_per_row = w // 8

        self._cmd(0x24)  # WRITE_RAM_BW
        self._cs(0)
        self._dc(1)

        for row in range(y, y + h):
            row_start = row * bytes_per_row
            window_start = row_start + (x // 8)
            window_end = window_start + window_bytes_per_row
            self._spi.write(self._buf[window_start:window_end])

        self._cs(1)

        # Trigger partial update
        self._update_part()

    def show_partial_advanced(self, x, y, w, h):
        """
        Advanced partial update with arbitrary pixel alignment
        Handles non-byte-aligned x positions through bit manipulation

        Args:
            x: X position (any value 0-399)
            y: Y position (any value 0-299)
            w: Width in pixels (any value)
            h: Height in pixels (any value)
        """
        # Validate bounds
        if x < 0 or y < 0 or x + w > self._w or y + h > self._h:
            raise ValueError("Partial window out of display bounds")

        # Calculate byte-aligned region (may be slightly larger than requested)
        x_byte_aligned = (x // 8) * 8
        w_byte_aligned = ((x + w + 7) // 8) * 8 - x_byte_aligned

        # Configure for partial update
        self._cmd(0x3C); self._dat(0x80)
        self._cmd(0x21); self._dat(0x00); self._dat(0x00)
        self._cmd(0x3C); self._dat(0x80)
        self._cmd(0x11); self._dat(0x03)

        # Set address window and cursor
        self._pos(x_byte_aligned, y, x_byte_aligned + w_byte_aligned - 1, y + h - 1)
        self._cur(x_byte_aligned, y)

        # Stream region data
        bytes_per_row = self._w // 8
        window_bytes_per_row = w_byte_aligned // 8

        self._cmd(0x24)
        self._cs(0)
        self._dc(1)

        for row in range(y, y + h):
            row_start = row * bytes_per_row
            window_start = row_start + (x_byte_aligned // 8)
            window_end = window_start + window_bytes_per_row
            self._spi.write(self._buf[window_start:window_end])

        self._cs(1)
        self._update_part()

    # ============ Utility Methods ============

    def sleep(self):
        """
        Put display into deep sleep mode (low power)
        Call init() or init_fast() to wake up
        """
        self._cmd(0x10)  # DEEP_SLEEP_MODE
        self._dat(0x01)  # Enter deep sleep
        sleep_ms(100)

    def fill_rect_partial(self, x, y, w, h, color, update=True):
        """
        Fill a rectangle and update only that region
        Convenience method combining drawing and partial update

        Args:
            x, y: Top-left corner (x should be multiple of 8)
            w, h: Dimensions (w should be multiple of 8)
            color: 0 (black) or 1 (white)
        """
        # Draw to framebuffer
        self.fill_rect(x, y, w, h, color)
        # Update only that region
        if update:
            self.show_partial(x, y, w, h)

    def text_partial(self, text, x, y, color=0, update=True):
        """
        Draw text and update only the affected region
        Note: Requires x to be aligned to 8 pixels

        Args:
            text: Text string
            x, y: Position (x should be multiple of 8)
            color: Text color (0=black, 1=white)
        """
        # Estimate text width (8 pixels per char)
        est_width = len(text) * 8
        # Round up to multiple of 8
        w = ((est_width + 7) // 8) * 8
        h = 8  # Standard font height

        # Draw text
        self.text(text, x, y, color)
        # Partial update
        if update:
            self.show_partial(x, y, w, h)


# ============ Usage Example ============

def example_usage():
    """
    Example demonstrating all update modes
    """
    # Initialize display
    epd = SSD1683()

    # Method 1: Full clear (important before partial updates)
    print("Clearing display...")
    epd.clear()  # Syncs both internal RAMs

    # Method 2: Full refresh (slow, clean)
    print("Full refresh...")
    epd.fill(1)  # White
    epd.text("Full Refresh", 10, 10, 0)
    epd.show()  # ~2-3 seconds

    # Method 3: Fast refresh (faster full screen)
    print("Fast refresh...")
    epd.fill(1)
    epd.text("Fast Refresh", 10, 30, 0)
    epd.show_fast()  # ~1.5 seconds

    # Method 4: Partial refresh (fastest)
    print("Partial refresh...")
    # Note: x and w are multiples of 8
    epd.fill_rect(0, 50, 200, 20, 1)  # Clear region
    epd.text("Partial Update", 0, 50, 0)
    epd.show_partial(0, 50, 200, 20)  # ~0.3 seconds

    # Multiple partial updates
    for i in range(5):
        epd.fill_rect(0, 100, 200, 20, 1)
        epd.text(f"Count: {i}", 0, 100, 0)
        epd.show_partial(0, 100, 200, 20)
        sleep_ms(500)

    # Do a full refresh every N partial updates to clear ghosting
    epd.show()

    # Sleep when done
    epd.sleep()


# ============ Best Practices ============

"""
BEST PRACTICES FOR USING THIS DRIVER:

1. INITIALIZATION:
   - Use init() for general purpose
   - Use init_fast() if you'll mainly do fast/partial updates

2. CLEARING:
   - Always call clear() before starting partial updates
   - This synchronizes internal RAMs (0x24 and 0x26)

3. UPDATE STRATEGY:
   - Full update (show()): Use occasionally to remove ghosting
   - Fast update (show_fast()): For frequent full-screen changes
   - Partial update (show_partial()): For small region updates

4. GHOSTING MANAGEMENT:
   - Partial updates accumulate ghosting
   - Do a full update every 5-10 partial updates
   - Or call show() whenever ghosting becomes visible

5. PARTIAL UPDATE CONSTRAINTS:
   - x and w should be multiples of 8 for show_partial()
   - Use show_partial_advanced() for arbitrary positions
   - Plan your UI layout with 8-pixel columns in mind

6. POWER MANAGEMENT:
   - Call sleep() when display won't be updated for a while
   - Wake with init() or init_fast()

7. TYPICAL WORKFLOW:
   epd = SSD1683()
   epd.clear()                    # Initial clear

   # Fast updates
   for i in range(10):
       epd.fill_rect(0, 0, 200, 50, 1)
       epd.text(f"Value: {i}", 0, 0, 0)
       epd.show_partial(0, 0, 200, 50)

   epd.show()                     # Clean up ghosting
   epd.sleep()                    # Save power
"""
