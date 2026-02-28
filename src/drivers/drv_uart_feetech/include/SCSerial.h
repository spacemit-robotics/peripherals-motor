
#ifndef SCSERIAL_H
#define SCSERIAL_H

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/select.h>
#include <cstdint>
#include "SCS.h"

class SCSerial : public SCS {
public:
    SCSerial();
    explicit SCSerial(u8 End);
    SCSerial(u8 End, u8 Level);

protected:
    int writeSCS(unsigned char *nDat, int nLen);  // 输出nLen字节
    int readSCS(unsigned char *nDat, int nLen);  // 输入nLen字节
    int readSCS(unsigned char *nDat, int nLen, uint32_t TimeOut);
    int writeSCS(unsigned char bDat);  // 输出1字节
    void rFlushSCS();
    void wFlushSCS();

public:
    uint32_t IOTimeOut;  // 输入输出超时

public:
    int setBaudRate(int baudRate);
    bool begin(int baudRate, const char* serialPort);
    void end();

protected:
    int fd;  // serial port handle
    struct termios orgopt;  // fd ort opt
    struct termios curopt;  // fd cur opt
    unsigned char txBuf[255];
    int txBufLen;
};

#endif  // SCSERIAL_H
