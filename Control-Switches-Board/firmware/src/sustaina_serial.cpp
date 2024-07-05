#include "sustaina_serial.h"
#include "config.h"
#include "PmxCRC/PmxCRC.h"

PmxCrc16 CRC;

void SUSTAINA_PMX_SERIAL::setupSerial(){
  RS485_SERIAL.begin(RS485_SERIAL_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_TXDEN_PIN, OUTPUT);
}

bool SUSTAINA_PMX_SERIAL::checkHeader() {
  for (size_t i = 0; i < HEADER_LENGTH; i++) {
    if (RS485_SERIAL.read() != HEADER[i]) {
      return false;
    }
    rxPacket[i] = HEADER[i];
  }
  return true;
}

bool SUSTAINA_PMX_SERIAL::readPacket() {
  if (RS485_SERIAL.available() >= RX_PACKET_MIN_LENGTH) {
    rxData.clear();
    txData.clear();
    rxPacket.clear();
    txError = 0b00000000;

    rxPacket.resize(RX_PACKET_FORWARD_LENGTH);

    if (checkHeader()) {
      for (int i = HEADER_LENGTH; i < RX_PACKET_FORWARD_LENGTH; i++) {
        rxPacket[i] = RS485_SERIAL.read();
      }

      size_t rxPacketIndex = HEADER_LENGTH;
      rxId = rxPacket[rxPacketIndex++];
      rxLength = rxPacket[rxPacketIndex++];
      rxCommand = rxPacket[rxPacketIndex++];
      rxOption = rxPacket[rxPacketIndex++];

      // Resize rxPacket to hold the complete packet
      rxPacket.resize(rxLength);

      // Read the rest of the packet
      for (size_t i = RX_PACKET_FORWARD_LENGTH; i < rxLength; i++) {
        rxPacket[i] = RS485_SERIAL.read();
      }

      rxDataLength = rxLength - RX_PACKET_FORWARD_LENGTH - CRC_LENGTH;
      
      if (rxDataLength > 0){
        rxData.resize(rxDataLength);
        for (size_t i = 0; i < rxLength - CRC_LENGTH; i++) {
          rxData[i] = rxPacket[i + RX_PACKET_FORWARD_LENGTH];
        }
      }

      return true;
    }
  }
  return false;
}

bool SUSTAINA_PMX_SERIAL::checkCRCandID() {
  if (rxId != BOARD_ID)
    return false;

  if (CRC.checkCrc16(const_cast<uint8_t*>(rxPacket.data())))
    return true;

  txError |= CRC_ERROR;
  return false;
}

bool SUSTAINA_PMX_SERIAL::commandProcess() {
  switch (rxCommand) {
      case CHECK_FIRMWARE_CMD:
        txData.push_back(FIRMWARE_VERSION);
        return true;

      case CPU_RESET_CMD:
        ESP.restart();
        return true;
        
      default:
        return false;
  }
}

void SUSTAINA_PMX_SERIAL::sendPacket() {
  size_t txPacket_length = TX_PACKET_MIN_LENGTH + txData.size();
  vector<uint8_t> txPacket(txPacket_length);

  // Fill txPacket
  memcpy(txPacket.data(), HEADER, HEADER_LENGTH);
  size_t txPacketIndex = HEADER_LENGTH;

  txPacket[txPacketIndex++] = BOARD_ID;
  txPacket[txPacketIndex++] = static_cast<uint8_t>(txPacket_length);
  txPacket[txPacketIndex++] = rxCommand & RETURN_CMD_MASK;  // Command
  txPacket[txPacketIndex++] = txError;               // Error

  // Add txData to txPacket
  if (!txData.empty()) {
    memcpy(txPacket.data() + txPacketIndex, txData.data(), txData.size());
    txPacketIndex += txData.size();
  }

  // Add CRC to txPacket
  uint16_t txCrc = CRC.getCrc16(txPacket.data(), txPacket_length - CRC_LENGTH);
  txPacket[txPacketIndex++] = lowByte(txCrc);
  txPacket[txPacketIndex++] = highByte(txCrc);

  // Send data
  digitalWrite(RS485_TXDEN_PIN, HIGH);
  RS485_SERIAL.write(txPacket.data(), txPacket_length);
  RS485_SERIAL.flush();
  digitalWrite(RS485_TXDEN_PIN, LOW);
}
