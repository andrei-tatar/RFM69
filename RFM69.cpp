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
#include "RFM69.h"
#include "RFM69registers.h"
#include <string.h>

RFM69::RFM69(spiTransferFunction spiTransfer, getTimeFunction getTime, bool isRFM69HW)
    : _spiTransfer(spiTransfer), _getTime(getTime), _isRFM69HW(isRFM69HW)
{
  _mode = RF69_MODE_STANDBY;
  _powerLevel = 31;
}

bool RFM69::initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{
  const uint8_t CONFIG[][2] =
      {
          /* 0x01 */ {REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY},
          /* 0x02 */ {REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00}, // no shaping
          /* 0x03 */ {REG_BITRATEMSB, RF_BITRATEMSB_55555},                                                                               // default: 4.8 KBPS
          /* 0x04 */ {REG_BITRATELSB, RF_BITRATELSB_55555},
          /* 0x05 */ {REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
          /* 0x06 */ {REG_FDEVLSB, RF_FDEVLSB_50000},

          /* 0x07 */ {REG_FRFMSB, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFMSB_315 : (freqBand == RF69_433MHZ ? RF_FRFMSB_433 : (freqBand == RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915)))},
          /* 0x08 */ {REG_FRFMID, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFMID_315 : (freqBand == RF69_433MHZ ? RF_FRFMID_433 : (freqBand == RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915)))},
          /* 0x09 */ {REG_FRFLSB, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFLSB_315 : (freqBand == RF69_433MHZ ? RF_FRFLSB_433 : (freqBand == RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915)))},

          // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
          // +17dBm and +20dBm are possible on RFM69HW
          // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
          // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
          // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
          ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
          ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

          // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
          /* 0x19 */ {REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2}, // (BitRate < 2 * RxBw)
                                                                                        //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
          /* 0x25 */ {REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01},                         // DIO0 is the only IRQ we're using
          /* 0x26 */ {REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF},                      // DIO5 ClkOut disable for power saving
          /* 0x28 */ {REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN},                         // writing to this bit ensures that the FIFO & status flags are reset
          /* 0x29 */ {REG_RSSITHRESH, 220},                                             // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
                                                                                        ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
          /* 0x2E */ {REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0},
          /* 0x2F */ {REG_SYNCVALUE1, 0x2D},      // attempt to make this compatible with sync1 byte of RFM12B lib
          /* 0x30 */ {REG_SYNCVALUE2, networkID}, // NETWORK ID
          /* 0x37 */ {REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODE},
          /* 0x38 */ {REG_PAYLOADLENGTH, 66}, // in variable length mode: the max frame size, not used in TX
          /* 0x39 */ {REG_NODEADRS, nodeID},
          /* 0x3C */ {REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE},                              // TX on FIFO not empty
          /* 0x3D */ {REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF}, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
                                                                                                                              //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
          /* 0x6F */ {REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0},                                                               // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
          {255, 0}};

  unsigned long start = _getTime();
  uint8_t timeout = 50;
  do
    writeReg(REG_SYNCVALUE1, 0xAA);
  while (readReg(REG_SYNCVALUE1) != 0xaa && _getTime() - start < timeout);
  start = _getTime();
  do
    writeReg(REG_SYNCVALUE1, 0x55);
  while (readReg(REG_SYNCVALUE1) != 0x55 && _getTime() - start < timeout);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  encrypt(0);

  setHighPower(); // called regardless if it's a RFM69W or RFM69HW
  setMode(RF69_MODE_STANDBY);
  start = _getTime();
  while (!isModeReady() && _getTime() - start < timeout)
    ; // wait for ModeReady
  if (_getTime() - start >= timeout)
    return false;

  _address = nodeID;
  return true;
}

// return the frequency (in Hz)
uint32_t RFM69::getFrequency()
{
  return RF69_FSTEP * (((uint32_t)readReg(REG_FRFMSB) << 16) + ((uint16_t)readReg(REG_FRFMID) << 8) + readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void RFM69::setFrequency(uint32_t freqHz)
{
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX)
  {
    setMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  writeReg(REG_FRFMSB, freqHz >> 16);
  writeReg(REG_FRFMID, freqHz >> 8);
  writeReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX)
  {
    setMode(RF69_MODE_SYNTH);
  }
  setMode(oldMode);
}

void RFM69::setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  switch (newMode)
  {
  case RF69_MODE_TX:
    updateReg(REG_OPMODE, 0xE3, RF_OPMODE_TRANSMITTER);
    if (_isRFM69HW)
      setHighPowerRegs(true);
    break;
  case RF69_MODE_RX:
    updateReg(REG_OPMODE, 0xE3, RF_OPMODE_RECEIVER);
    if (_isRFM69HW)
      setHighPowerRegs(false);
    break;
  case RF69_MODE_SYNTH:
    updateReg(REG_OPMODE, 0xE3, RF_OPMODE_SYNTHESIZER);
    break;
  case RF69_MODE_STANDBY:
    updateReg(REG_OPMODE, 0xE3, RF_OPMODE_STANDBY);
    break;
  case RF69_MODE_SLEEP:
    updateReg(REG_OPMODE, 0xE3, RF_OPMODE_SLEEP);
    break;
  default:
    return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && !isModeReady())
    ; // wait for ModeReady

  _mode = newMode;
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void RFM69::sleep()
{
  setMode(RF69_MODE_SLEEP);
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void RFM69::setPowerLevel(uint8_t powerLevel)
{
  _powerLevel = (powerLevel > 31 ? 31 : powerLevel);
  if (_isRFM69HW)
    _powerLevel /= 2;
  updateReg(REG_PALEVEL, 0xE0, _powerLevel);
}

bool RFM69::canSend()
{
  if (_mode == RF69_MODE_RX && readRSSI() < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void RFM69::send(uint8_t toAddress, const uint8_t *buffer, uint8_t bufferSize)
{
  updateReg(REG_PACKETCONFIG2, 0xFB, RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = _getTime();
  while (!canSend() && _getTime() - now < RF69_CSMA_LIMIT_MS)
  {
    // wait until the channel is silent
  }
  sendFrame(toAddress, buffer, bufferSize);
}

// internal function
void RFM69::sendFrame(uint8_t toAddress, const uint8_t *buffer, const uint8_t bufferSize)
{
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while (!isModeReady())
    ;                                                // wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN)
    return;

  // write to FIFO
  uint8_t data[4 + bufferSize];
  data[0] = REG_FIFO | 0x80;
  data[1] = bufferSize + 3;
  data[2] = toAddress;
  data[3] = _address;
  memcpy(&data[4], buffer, bufferSize);
  _spiTransfer(data, sizeof(data));

  _packetSent = false;
  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  uint32_t txStart = _getTime();
  while (!_packetSent && _getTime() - txStart < RF69_TX_LIMIT_MS) {
    // wait for DIO0 to turn HIGH signalling transmission finish
  }
  setMode(RF69_MODE_STANDBY);
}

void RFM69::interrupt(RfmPacket &packet)
{
  uint8_t irqFlags = readReg(REG_IRQFLAGS2);

  if (_mode == RF69_MODE_RX && (irqFlags & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    uint8_t header[4] = {REG_FIFO & 0x7F};
    _spiTransfer(header, sizeof(header));

    uint8_t payloadLength = header[1];
    if (payloadLength > 66)
      payloadLength = 66;
    //uint8_t target = header[2];
    packet.size = payloadLength - 3;
    packet.from = header[3];

    packet.data[0] = REG_FIFO & 0x7F;
    _spiTransfer(packet.data, packet.size + 1);
    for (uint8_t i = 0; i < packet.size; i++)
      packet.data[i] = packet.data[i + 1];
  }
  else if (_mode == RF69_MODE_TX && (irqFlags & RF_IRQFLAGS2_PACKETSENT))
  {
    _packetSent = true;
  }

  packet.rssi = readRSSI();
}

void RFM69::receiveBegin()
{
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    updateReg(REG_PACKETCONFIG2, 0xFB, RF_PACKET2_RXRESTART); // avoid RX deadlocks
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);          // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void RFM69::encrypt(const char *key)
{
  setMode(RF69_MODE_STANDBY);
  if (key != 0)
  {
    uint8_t rfmKey[17] = {REG_AESKEY1 | 0x80};
    memcpy(&rfmKey[1], key, 16);
    _spiTransfer(rfmKey, sizeof(rfmKey));
  }
  updateReg(REG_PACKETCONFIG2, 0xFE, (key ? 1 : 0));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69::readRSSI(bool forceTrigger)
{
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00)
      ; // wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t RFM69::readReg(uint8_t addr)
{
  uint8_t rd[] = {(uint8_t)(addr & 0x7F), 0};
  _spiTransfer(rd, sizeof(rd));
  return rd[1];
}

void RFM69::writeReg(uint8_t addr, uint8_t value)
{
  uint8_t wr[] = {(uint8_t)(addr | 0x80), value};
  _spiTransfer(wr, sizeof(wr));
}

void RFM69::updateReg(uint8_t addr, uint8_t mask, uint8_t flags)
{
  uint8_t read = readReg(addr) & mask;
  writeReg(addr, read | flags);
}

// for RFM69HW only: you must call setHighPower(true) after initialize() or else transmission won't work
void RFM69::setHighPower()
{
  writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW)                                                        // turning ON
    updateReg(REG_PALEVEL, 0x1F, RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

// internal function
void RFM69::setHighPowerRegs(bool onOff)
{
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

uint8_t RFM69::readTemperature(uint8_t calFactor) // returns centigrade
{
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING))
    ;
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
}

void RFM69::rcCalibration()
{
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00)
    ;
}

inline bool RFM69::isModeReady()
{
  return (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) != 0;
}
