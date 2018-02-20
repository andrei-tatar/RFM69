// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h

#include <stdint.h>

#define RF69_MAX_DATA_LEN 61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 2 bytes overhead - 2 bytes crc)

#define CSMA_LIMIT 180      // upper RX signal sensitivity threshold (for dBm: -x/2; eg: 180 = -90dBM)
#define RF69_MODE_SLEEP 0   // XTAL OFF
#define RF69_MODE_STANDBY 1 // XTAL ON
#define RF69_MODE_SYNTH 2   // PLL ON
#define RF69_MODE_RX 3      // RX MODE
#define RF69_MODE_TX 4      // TX MODE

// available frequency bands
#define RF69_315MHZ 31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ 43
#define RF69_868MHZ 86
#define RF69_915MHZ 91

#define COURSE_TEMP_COEF -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS 1000
#define RF69_FSTEP 61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

typedef void (*spiTransferFunction)(uint8_t *data, uint8_t len);
typedef unsigned long (*getTimeFunction)();
typedef struct
{
  uint8_t data[RF69_MAX_DATA_LEN];
  uint8_t rssi;
  uint8_t from;
  uint8_t size;
} RfmPacket;

class RFM69
{
public:
  RFM69(spiTransferFunction spiTransfer, getTimeFunction getTime, bool isRFM69HW = false);
  bool initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID = 1);

  bool canSend();
  void send(uint8_t toAddress, const uint8_t *buffer, uint8_t bufferSize);
  uint32_t getFrequency();
  void setFrequency(uint32_t freqHz);
  void encrypt(const char *key);
  uint8_t readRSSI(bool forceTrigger = false);
  void promiscuous(bool onOff = true);
  void setPowerLevel(uint8_t level); // reduce/increase transmit power level
  void sleep();
  uint8_t readTemperature(uint8_t calFactor = 0); // get CMOS temperature (8bit)
  void rcCalibration();                           // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

  void interrupt(RfmPacket &packet);
  void receiveBegin();

private:
  inline bool isModeReady();
  void setHighPower();
  void setMode(uint8_t mode);
  void setHighPowerRegs(bool onOff);
  void sendFrame(uint8_t toAddress, const uint8_t *buffer, uint8_t size);
  uint8_t readReg(uint8_t addr);
  void writeReg(uint8_t addr, uint8_t val);
  void updateReg(uint8_t addr, uint8_t mask, uint8_t flags);

  const spiTransferFunction _spiTransfer;
  const getTimeFunction _getTime;
  const bool _isRFM69HW;

  uint8_t _address;
  uint8_t _powerLevel;

  volatile bool _packetSent;
  volatile uint8_t _mode; // should be protected?
};

#endif
