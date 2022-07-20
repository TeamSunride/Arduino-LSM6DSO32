# Arduino-LSM6DSO32
Arduino library for the LSM6DSO32 inertial module https://www.st.com/en/mems-and-sensors/lsm6dso32.html


# Usage
Use the appropriate constructor for your configuration:
- SPI constructor: (recommended)
```cpp
#define CS_pin 10 // e.g.
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
LSM6DS032 LSM(CS_pin, SPI, settings);
```
- I2C constructor:
```cpp
LSM6DS032 LSM(&Wire, 1000000);
```

~~fun~~

Wiring guide:
```mermaid
graph TD;
    mcu[MCU] -- SCK  ---LSM6DSO32[LSM6DSO32];
    mcu-- MOSI  ---LSM6DSO32[LSM6DSO32]
    mcu-- MISO  ---LSM6DSO32[LSM6DSO32]
    mcu-- CS  ---LSM6DSO32[LSM6DSO32] 
    
    mcu2[MCU] -- SCL  ---LSM6DSO32_2[LSM6DSO32];
    mcu2 -- SDA  ---LSM6DSO32_2[LSM6DSO32]
```



With the correct configuration (see `default_configuration()`) calling `fifo_pop` will pop from the LSM FIFO into the acceleration and gyroscope FIFOs.
Note that ACC FIFO and GYRO FIFO are FIFOs of _type_ `Vector<double, 3>`, so they are FIFOs of Vectors.
```mermaid
graph TD;
    rawData[Raw Sensor Data]-- Continuously Streaming ---lsmFifo[LSM FIFO];
    lsmFifo--> accFifo[ACC FIFO] & gyroFifo[GYRO FIFO];
   
```

