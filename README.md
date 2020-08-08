# ESP32 Electric UI Example Project

Basic project demonstrating integration of the electricui-embedded library with ESP32 using ESP-IDF.

Developed against a [Sparkfun ESP32 Thing Plus](https://www.sparkfun.com/products/15663).

# Development

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