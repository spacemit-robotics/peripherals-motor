
#include "SCSerial.h"

#include <cstdio>

SCSerial::SCSerial()
{
    IOTimeOut = 100;  // 100ms 超时
    fd = -1;
    txBufLen = 0;
}

SCSerial::SCSerial(u8 End):SCS(End)
{
    IOTimeOut = 100;  // 100ms 超时
    fd = -1;
    txBufLen = 0;
}

SCSerial::SCSerial(u8 End, u8 Level):SCS(End, Level)
{
    IOTimeOut = 100;  // 100ms 超时
    fd = -1;
    txBufLen = 0;
}

bool SCSerial::begin(int baudRate, const char *serialPort) {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
    if (serialPort == NULL) return false;
    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        perror("open:");
        return false;
    }
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);
    speed_t CR_BAUDRATE;
    switch (baudRate) {
        case 9600:
            CR_BAUDRATE = B9600;
            break;
        case 19200:
            CR_BAUDRATE = B19200;
            break;
        case 38400:
            CR_BAUDRATE = B38400;
            break;
        case 57600:
            CR_BAUDRATE = B57600;
            break;
        case 115200:
            CR_BAUDRATE = B115200;
            break;
        case 500000:
            CR_BAUDRATE = B500000;
            break;
        case 1000000:
            CR_BAUDRATE = B1000000;
            break;
        default:
            CR_BAUDRATE = B115200;
            break;
    }
    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);

    printf("serial speed %d\n", baudRate);
    // Mostly 8N1
    curopt.c_cflag &= ~PARENB;
    curopt.c_cflag &= ~CSTOPB;
    curopt.c_cflag &= ~CSIZE;
    curopt.c_cflag |= CS8;
    curopt.c_cflag |= CREAD;
    curopt.c_cflag |= CLOCAL;  // disable modem statuc check
    cfmakeraw(&curopt);        // make raw mode
    curopt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    if (tcsetattr(fd, TCSANOW, &curopt) == 0) {
        // 彻底清空缓冲区
        tcflush(fd, TCIOFLUSH);
        usleep(100000);  // 100ms 等待串口稳定

        // 读取并丢弃所有残留数据
        char discard[256];
        int total_discarded = 0;
        while (1) {
            fd_set fds;
            struct timeval tv;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = 10000;  // 10ms
            int ret = select(fd + 1, &fds, NULL, NULL, &tv);
            if (ret > 0) {
                int n = read(fd, discard, sizeof(discard));
                if (n > 0)
                    total_discarded += n;
                else
                    break;
            } else {
                break;
            }
        }
        if (total_discarded > 0) {
            printf("Discarded %d bytes of stale data\n", total_discarded);
        }

        tcflush(fd, TCIOFLUSH);  // 最后再清空一次
        return true;
    } else {
        perror("tcsetattr:");
        return false;
    }
}

int SCSerial::setBaudRate(int baudRate) {
    if (fd == -1) {
        return -1;
    }
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);
    speed_t CR_BAUDRATE;
    switch (baudRate) {
        case 9600:
            CR_BAUDRATE = B9600;
            break;
        case 19200:
            CR_BAUDRATE = B19200;
            break;
        case 38400:
            CR_BAUDRATE = B38400;
            break;
        case 57600:
            CR_BAUDRATE = B57600;
            break;
        case 115200:
            CR_BAUDRATE = B115200;
            break;
        case 230400:
            CR_BAUDRATE = B230400;
            break;
        case 500000:
            CR_BAUDRATE = B500000;
            break;
        default:
            break;
    }
    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);
    return 1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen, uint32_t TimeOut) {
    int fs_sel;
    fd_set fs_read;
    int rvLen = 0;

    struct timeval time;

    // 使用select实现串口的多路通信
    while (1) {
        FD_ZERO(&fs_read);
        FD_SET(fd, &fs_read);

        // 每次循环重新设置超时时间（select会修改timeval）
        time.tv_sec = 0;
        time.tv_usec = TimeOut * 1000;

        fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
        if (fs_sel) {
            rvLen += read(fd, nDat + rvLen, nLen - rvLen);
            // printf("nLen = %d rvLen = %d\n", nLen, rvLen);
            if (rvLen < nLen) {
                continue;
            } else {
                return rvLen;
            }
        } else {
            // printf("serial read fd read return 0\n");
            return rvLen;
        }
    }
}

int SCSerial::readSCS(unsigned char *nDat, int nLen) {
    int fs_sel;
    fd_set fs_read;
    int rvLen = 0;
    int retryCount = 0;
    const int maxRetry = 10;

    struct timeval time;

    while (retryCount < maxRetry) {
        FD_ZERO(&fs_read);
        FD_SET(fd, &fs_read);

        time.tv_sec = 0;
        time.tv_usec = IOTimeOut * 1000;

        fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
        if (fs_sel > 0) {
            int n = read(fd, nDat + rvLen, nLen - rvLen);
            if (n > 0) {
                rvLen += n;
                retryCount = 0;
            } else if (n == 0 || (n < 0 && errno == EAGAIN)) {
                retryCount++;
                usleep(1000);  // 1ms
                continue;
            }
            if (rvLen >= nLen) {
                return rvLen;
            }
            continue;
        } else {
            if (rvLen > 0) {
                retryCount++;
                continue;
            }
            return rvLen;
        }
    }
    return rvLen;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen) {
    while (nLen--) {
        txBuf[txBufLen++] = *nDat++;
    }
    return txBufLen;
}

int SCSerial::writeSCS(unsigned char bDat) {
    txBuf[txBufLen++] = bDat;
    return txBufLen;
}

void SCSerial::rFlushSCS() { tcflush(fd, TCIFLUSH); }

void SCSerial::wFlushSCS() {
    if (txBufLen) {
        ssize_t written = write(fd, txBuf, txBufLen);
        (void)written;
        tcdrain(fd);  // 等待发送完成
        txBufLen = 0;
    }
}

void SCSerial::end() {
    fd = -1;
    close(fd);
}
