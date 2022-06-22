#include <Arduino.h>
#include "device.h"
#include "LSMDS032_registers.h"
#include <SPI.h>
#include "device.h"

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
*/

/* Defines */
#define CS 35
#define MOSI 11
#define MISO 12
#define SCK 13 // LED is also on this pin
#define WRITE 0b10000000
#define READ 0b00000000



void setup() {
    pinMode(CS, OUTPUT); // CS is pulled low when selected
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);

    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
}


int REGREAD(byte address, int bytesToRead) {  // FIFO
    address = READ | address; // puts 0 int the 8th bit.
    byte inByte = 0;
    int result = 0;
    digitalWrite(CS, LOW); // begin transfer
    for (int i = 0; i < bytesToRead; i++) {
        result = result << 8;
        inByte = SPI.transfer(0x00);  // transfers 0x00 over MOSI line, recieves a byte over MISO line.
        result = result | inByte;
    }
    digitalWrite(CS, HIGH); // end transfer
    return result;
}


void REGSET(byte address, byte value) {
    address = WRITE | address; //
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
}


void loop() {

}
