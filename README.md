# Tidepool-Junior

Zephyr blinky for the Teensy 4.0. The onboard LED (pin 13, `led0`)
toggles every 1000 ms. `CONFIG_GPIO=y` enables the GPIO driver; the board's
devicetree already defines the LED, so no overlay is needed.

From the parent workspace directory:

```sh
source .venv/bin/activate
west build -b teensy40 Tidepool-Junior -d build/junior
```

Ninja must be on PATH. It is installed in this workspace's `.venv`, so activate
the environment in each new terminal. To install it again using uv:

```sh
uv pip install --python .venv/bin/python ninja
```

Use `west build` so Zephyr selects the ARM SDK toolchain. The old workspace
`build` directory was configured against Zephyr itself with host Clang;
`build/junior` keeps this application's configuration separate.

The firmware is written to `build/junior/zephyr/zephyr.hex`. With
`teensy_loader_cli` installed and on PATH, connect the board over USB,
press its Program button to enter the bootloader, then run:

```sh
west flash -d build/junior
```

Alternatively, open that HEX file in the PJRC Teensy Loader application.

The default console uses UART pins 0 (RX) and 1 (TX) at 115200 baud, 8N1.
The LED works without a serial connection; `printf` output does not go to
USB serial with this configuration.
