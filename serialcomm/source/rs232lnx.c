#ifdef __linux__

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/signal.h>
#include <sys/types.h>
#include "rs232lnx.h"

static int fd;
static struct timeval tv;
static fd_set rfds;
T_RS232LNX_INSTANCE* instance;

unsigned char rs232lnxConfigure();
unsigned char rs232lnxFlush();
void rs232lnxSetTimeout(unsigned int timeout);

/**
 * Opens the serial port
 * @param _instance Structure that contains the information about the communication to be opened
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxOpen(T_RS232LNX_INSTANCE* _instance) {
    if(_instance == NULL) {
        return RS232LNX_ERROR_GENERIC;
    }
    
    instance = _instance;

    instance->device = open(instance->port, (O_RDWR | O_NOCTTY));
    if (instance->device == -1) {
        return RS232LNX_ERROR_GENERIC;
    }

    if(rs232lnxConfigure() == RS232LNX_OK) {
        instance->isActive = 1;
        return RS232LNX_OK;
    } else {
        return RS232LNX_ERROR_GENERIC;
    }    
}

/**
 * Configures the opened serial communication
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxConfigure() {
    // serial port configuration structure
    struct termios options;
    
    // get the current settings of the serial port
    if(tcgetattr(instance->device, &options) != 0) {
        return RS232LNX_ERROR_GENERIC;
    }

    //CLOCAL means don’t allow control of the port to be changed
    //CREAD says to enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    //CSIZE is a mask for all the data size bits, so anding with the negation clears out the current data size setting
    options.c_cflag &= ~CSIZE;

    // Speed (Read and Write). All speeds can be prefixed with B as a settings.
    switch (instance->speed) {

        case RS232LNX_SPEED_9600:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
            break;

        case RS232LNX_SPEED_115200:
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            break;

        default:
            return RS232LNX_ERROR_GENERIC;
            break;
    }

    // Parity
    switch (instance->parity) {

        case RS232LNX_PARITY_NONE:
            //PARENB is enable parity bit
            //so this disables the parity bit
            //TODO options.c_cflag &= ~PARENB;
            options.c_cflag &= ~(PARENB | PARODD);
            break;

        case RS232LNX_PARITY_EVEN:
            //PARENB is enable parity bit
            //so this disables the parity bit
            options.c_cflag |= PARENB;
            break;

        case RS232LNX_PARITY_ODD:
            //PARENB is enable parity bit
            //so this disables the parity bit
            options.c_cflag |= (PARENB | PARODD);
            break;

        default:
            return RS232LNX_ERROR_GENERIC;
            break;
    }

    // Stopbits
    switch (instance->stopBits) {

        case RS232LNX_STOPBITS_1:
            //CSTOPB means 2 stop bits otherwise (in this case) only one stop bit
            options.c_cflag &= ~CSTOPB;
            break;

        case RS232LNX_STOPBITS_2:
            //CSTOPB means 2 stop bits
            options.c_cflag |= CSTOPB;
            break;

        default:
            return RS232LNX_ERROR_GENERIC;
            break;
    }

    // Databits
    switch (instance->dataBits) {

        case RS232LNX_DATABITS_5:
            //CS5 means 5-bits per work
            options.c_cflag |= CS5;
            break;

        case RS232LNX_DATABITS_6:
            //CS6 means 6-bits per work
            options.c_cflag |= CS6;
            break;

        case RS232LNX_DATABITS_7:
            //CS7 means 7-bits per work
            options.c_cflag |= CS7;
            break;

        case RS232LNX_DATABITS_8:
            //CS8 means 8-bits per work
            options.c_cflag |= CS8;
            break;

        default:
            return RS232LNX_ERROR_GENERIC;
            break;
    }

    //Set the timeouts
    //VMIN is the minimum amount of characters to read.
    options.c_cc[VMIN] = 0;

    //The amount of time to wait for the amount of data specified by VMIN in tenths of a second.
    options.c_cc[VTIME] = (instance->timeout / 100);

    // Flushes the serial
    rs232lnxFlush();
    
    // Set the other configs
    options.c_iflag     = 0;
    options.c_oflag     = 0;
    options.c_lflag     = 0;
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    //apply the settings to the serial port
    // TCSNOW means apply the changes now other valid options include:
    // TCSADRAIN - wait until every thing has been transmitted
    // TCSAFLUSH - flush buffers and apply changes
    if (tcsetattr(instance->device, TCSANOW, &options) != 0) {
        //error code goes here
        return RS232LNX_ERROR_GENERIC;
    }

    return RS232LNX_OK;
}

/**
 * Sends a byte array to the serial device
 * @param _bytes byte array to be written
 * @param _bytesSize size of the byte array
 * @param _bytesWritten number of bytes that was actually written
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxWrite(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesWritten) {
    unsigned int ret = 0;

    if (_bytes == NULL) {
        return RS232LNX_ERROR_GENERIC;
    }

    if (_bytesWritten == NULL) {
        return RS232LNX_ERROR_GENERIC;
    }
    
    rs232lnxFlush();

    *_bytesWritten = 0;
    ret = (unsigned int) write(instance->device, _bytes, _bytesSize);
    *_bytesWritten = ret;

    if(ret < 0) {
        return RS232LNX_ERROR_GENERIC;
    } else {
        return RS232LNX_OK;
    }
}

/**
 * Reads a byte array from the serial device.
 * @param _bytes array where the read bytes will be stored
 * @param _bytesSize number of bytes expected
 * @param _bytesRead number of bytes that was actually read
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxRead(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesRead) {
    int ret = 0;
    int byte = 0;

    if (_bytes == NULL) {
        return RS232LNX_ERROR_GENERIC;
    }

    if (_bytesRead == NULL) {
        return RS232LNX_ERROR_GENERIC;
    }

    if (instance->isActive) {
        *_bytesRead = 0;
        rs232lnxSetTimeout(instance->timeout);

        // Loop to read each byte
        do {
            // Start the timeout
            ret = select(instance->device + 1, &rfds, NULL, NULL, &tv);
            if (ret > 0) {
                // Reads the byte
                ret = read(instance->device, &_bytes[byte], 1);
                if (ret == 1) {
                    // byte read
                    (*_bytesRead)++;
                    byte++;
                    if (byte == _bytesSize) {
                        return RS232LNX_OK;
                    }
                }
            } else {
                return RS232LNX_ERROR_GENERIC;
            }

        } while (1);
    }

    return RS232LNX_ERROR_GENERIC;
}

/**
 * Closes the serial port.
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxClose() {
    //close the serial port
    if (close(instance->device) == -1) {
        return RS232LNX_ERROR_GENERIC;
    }

    instance->isActive = 0;
    return RS232LNX_OK;
}

/**
 * Flushes the bytes remained in the serial buffer
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxFlush() {
    if (instance->isActive) {
        // Clears serial buffer
        if (tcflush(instance->device, TCIFLUSH) != 0) {
            return RS232LNX_ERROR_GENERIC;
        } else {
            return RS232LNX_OK;
        }
    }

    return RS232LNX_ERROR_GENERIC;
}

/**
 * Discards all characters from the output or input buffer of the connected serial device.
 * @param _clearType buffer to be cleared:
 *      RS232WIN_CLEAR_RX clears device input buffer
 *      RS232WIN_CLEAR_TX clears device output buffer
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232lnxClear(unsigned char _clearType)
{
    //TODO discard serial buffer data
    return RS232LNX_OK;
}

/**
 * Set the timeout counter
 * @param timeout timeout to be set
 */
void rs232lnxSetTimeout(unsigned int timeout) {
    timeout = ((timeout > 2147483647) ? 2147483647 : timeout);

    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000; //convert ms -> us

    FD_ZERO(&rfds);
    FD_SET(instance->device, &rfds);
}

#endif // __linux__