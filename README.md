## Flash micropython

```bash
uv run esptool --baud 460800 write_flash 0x1000 ESP32_GENERIC-SPIRAM-20251209-v1.27.0.bin
```

## Install typing
```bash
uv run mpremote mip install github:josverl/micropython-stubs/mip/typing_mpy.json
```