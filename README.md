# ESP32 Electric UI Example Project

Basic project demonstrating integration of the electricui-embedded library with ESP32 using ESP-IDF. Compatible with [the quickstart tutorial](https://electricui.com/docs/quick-start/).

We use Espressif's UART driver for inbuilt ring-buffer & event implementation. A task waits for serial events, passing bytes to `eui_parse()` as they come in. When a response is formed by eUI, it calls the `serial_write()` callback, which wraps the hal `uart_write_bytes()` call.

A separate task handles the simple user configurable blinking led common to the other eUI example demos. It runs every 10ms, checking if it should toggle the LED state or not, and updates the pin's state.

## Hardware Setup

- Developed against a [Sparkfun ESP32 Thing Plus](https://www.sparkfun.com/products/15663).
- Exposes eui access over `uart2` on TX1(17)/RX1(16) labelled pins on the Sparkfun board. Connect an (optional) external USB-UART adapter to these pins (and ground).
- Alternatively, use TX(1)/RX(3) to connect over the onboard USB connector. This requires no additional hardware to use.
- UARTs is configured to run at `115200` baud, 8N1 by default.
- The onboard blue LED (IO13) is used as a blinker for the standard [`hello-electric` example](https://electricui.com/docs/quick-start/ui).

## Firmware Setup

This is a normal ESP-IDF based example project and should work with your normal development toolchain. 

As this example is used with our CI/CD solution, the following instructions cover builds+flashing using the official [espressif/idf docker](https://hub.docker.com/r/espressif/idf/tags) container.

Running the toolchain interactively:

```
docker run -i --privileged --rm -v $PWD:/project -w /project -it espressif/idf:release-v4.2
```

Note we run the container in priviledged mode to get access to hardware. This is considered insecure and passing a specific serial port can be achieved with `-i --device=/dev/ttyUSB0` instead.

Once running in the container, `idf.py build`, and `idf.py flash` as normal.

## Dependencies

Relies on the `electricui-embedded` library, which can be downloaded from Github, uncompressed, and placed in `/components/electricui-embedded/`.

We introduce the library to the idf's component system, hence the `CMakeLists.txt` and `component.mk` in this repo at that location.

## Acknowledgements

This integration is essentially a minor variation of the official [UART Events Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/uart_events).