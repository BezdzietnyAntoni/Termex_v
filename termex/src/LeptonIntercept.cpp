#include "LeptonIntercept.h"

#include "SPI.h"
#include "Lepton_I2C.h"




#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)


LeptonIntercept::LeptonIntercept(){
  //loglevel - min value to return ERROR
  logLevel = 0;
  log = 0;
  currentLogMessage = "[Info] No errors";

  //Lepton model set v1.4
  myImageWidth = IMAGE_COLUMNS;
  myImageHeight = IMAGE_ROWS;

  //SPI speed - default 20Mhz
  spiSpeed = 20 * 1000 * 1000;

  //default value for connected==false
  connected = false;

}

LeptonIntercept::~LeptonIntercept() {
  //TODO If connected with spi disconect
    if (!connected)
        return;
    SpiClosePort(0);
}

void LeptonIntercept::performFFC(){
	//perform FFC
	lepton_perform_ffc();
}

void LeptonIntercept::setSpiSpeedMHz(unsigned int newSpiSpeed){
  spiSpeed = newSpiSpeed * 1000 * 1000;
}

void LeptonIntercept::connect(){
  SpiOpenPort(0, spiSpeed);
  //To do check connection
  connected = true;
}


void LeptonIntercept::getFrame(){
  int resets = 0;
  int segmentNumber = -1;
  uint16_t n_zero_value_drop_frame = 0;

  for(int j=0;j<PACKETS_PER_FRAME;j++) {
    //if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
    read(spi_cs0_fd, result+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
    int packetNumber = result[j*PACKET_SIZE+1];
    if(packetNumber != j) {
      j = -1;
      resets += 1;
      usleep(1000);
      //Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
      //By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
      if(resets == 750) {
        SpiClosePort(0);
        lepton_reboot();
        n_zero_value_drop_frame = 0;
        usleep(750000);
        SpiOpenPort(0, spiSpeed);
      }
      continue;
    }
  }
  if(resets >= 30) {
    saveLogMessage(3, "[WARNING] Done reading, resets: " + std::to_string(resets));
  }

  uint16_t valueFrameBuffer;
  int column;
  int row;
  for(int i=0;i<FRAME_SIZE_UINT16;i++) {
    //skip the first 2 uint16_t's of every packet, they're 4 header bytes
    if(i % PACKET_SIZE_UINT16 < 2) {
      continue;
    }
    //flip the MSB and LSB at the last second
    //data[i] = (result[i*2] << 8) + result[i*2+1];
    valueFrameBuffer = (result[i*2] << 8) + result[i*2+1];

    if (valueFrameBuffer == 0) {
      // Why this value is 0?
      n_zero_value_drop_frame++;
      if ((n_zero_value_drop_frame % 12) == 0) {
        saveLogMessage(5, "[WARNING] Found zero-value. Drop the frame continuously " + std::to_string(n_zero_value_drop_frame) + " times");
      }
      break;
    }

    column = (i % PACKET_SIZE_UINT16) - 2;
    row = i / PACKET_SIZE_UINT16;
    image_array[column+(row*IMAGE_COLUMNS)] = valueFrameBuffer;
  }

  if (n_zero_value_drop_frame != 0) {
    saveLogMessage(8, "[WARNING] Found zero-value. Drop the frame continuously " + std::to_string(n_zero_value_drop_frame) + " times [RECOVERED]");
    n_zero_value_drop_frame = 0;
  }
}


void LeptonIntercept::saveLogMessage(uint16_t level, std::string msg){
  if( log <= logLevel ){
    log = level;
    currentLogMessage = msg;
  }
}

bool LeptonIntercept::isLog(){
  return log;
}

std::string LeptonIntercept::getLog(){
  return currentLogMessage;
}
