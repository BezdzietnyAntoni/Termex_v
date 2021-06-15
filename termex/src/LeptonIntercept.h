#ifndef LEPTON_INTERCEPT
#define LEPTON_INTERCEPT

#include <stdint.h>
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <string>

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
#define IMAGE_ROWS 60
#define IMAGE_COLUMNS 80

class LeptonIntercept
{
public:
    LeptonIntercept();
    ~LeptonIntercept();

    void performFFC();
    void setSpiSpeedMHz(unsigned int);

    void connect();
    bool isConnected() const{return connected;};
    void getFrame(void);

    void saveLogMessage(uint16_t, std::string);
    bool isLog(void);
    std::string getLog(void);

    int getImageColumns(void) {return IMAGE_COLUMNS;};
    int getImageRows(void) {return IMAGE_ROWS;};
    uint16_t * getImagePointer(void) {return image_array;};


  private:
    bool connected;
    unsigned int spiSpeed;
    int myImageWidth;
    int myImageHeight;
    uint8_t result[PACKET_SIZE*PACKETS_PER_FRAME];
    uint16_t image_array[IMAGE_COLUMNS*IMAGE_ROWS];
    uint16_t logLevel = 0;
    uint16_t log = 0;
    std::string currentLogMessage;

};

#endif
