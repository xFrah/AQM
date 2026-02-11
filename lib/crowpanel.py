from machine import Pin, SDCard

SSD1683_WIDTH  = const(400)
SSD1683_HEIGHT = const(300)

SSD1683_PWR    = const(7)
SSD1683_CS     = const(45)
SSD1683_DC     = const(46)
SSD1683_RST    = const(47)
SSD1683_BUSY   = const(48)

SSD1683_SPI    = const(1)
SSD1683_SCK    = const(12)
SSD1683_MOSI   = const(11)

SDCARD_SLOT    = const(2)
SDCARD_PWR     = const(42)
SDCARD_CS      = const(10)
SDCARD_MOSI    = const(40)
SDCARD_CLK     = const(39)
SDCARD_MISO    = const(13)

KEY_HOME       = const(2)
KEY_EXIT       = const(1)
KEY_PREV       = const(6)
KEY_NEXT       = const(4)
KEY_DONE       = const(5)
POWER_LED      = const(41)

class CrowPanel42(object):

    def __init__(self):
        self._display = None
        self.home = Pin(KEY_HOME, Pin.IN)
        self.exit = Pin(KEY_EXIT, Pin.IN)
        self.prev = Pin(KEY_PREV, Pin.IN)
        self.done = Pin(KEY_DONE, Pin.IN)
        self.next = Pin(KEY_NEXT, Pin.IN)
        self.led = Pin(POWER_LED, Pin.OUT)

    def get_display(self):
        if self._display is None:
            import ssd1683
            Pin(SSD1683_PWR, Pin.OUT, value=1)
            self._display = ssd1683.SSD1683(SSD1683_WIDTH,
                                            SSD1683_HEIGHT,
                                            SSD1683_CS,
                                            SSD1683_DC,
                                            SSD1683_RST,
                                            SSD1683_BUSY,
                                            SSD1683_SPI,
                                            SSD1683_SCK,
                                            SSD1683_MOSI)
            # Initialize in fast mode for partial updates
            self._display.init_fast(mode_1s=True)
        return self._display

    def mount_sdcard(self, path):
        import vfs
        Pin(SDCARD_PWR, Pin.OUT, value=1)
        sd = SDCard(slot=SDCARD_SLOT,
                    sck=SDCARD_CLK,
                    miso=SDCARD_MISO,
                    mosi=SDCARD_MOSI,
                    cs=SDCARD_CS)
        vfs.mount(sd, path)
