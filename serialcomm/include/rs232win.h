#ifndef RS232WIN_H
#define	RS232WIN_H

#ifndef __linux__

#include "windows.h"

#define RS232WIN_SPEED_9600 9600
#define RS232WIN_SPEED_115200 115200

#define RS232WIN_PARITY_NONE 'n'
#define RS232WIN_PARITY_ODD 'o'
#define RS232WIN_PARITY_EVEN 'e'

#define RS232WIN_STOPBITS_1 '1'
#define RS232WIN_STOPBITS_2 '2'

#define RS232WIN_DATABITS_5 '5'
#define RS232WIN_DATABITS_6 '6'
#define RS232WIN_DATABITS_7 '7'
#define RS232WIN_DATABITS_8 '8'

#define RS232WIN_CLEAR_RX 0
#define RS232WIN_CLEAR_TX 1

#define RS232WIN_OK 0
#define RS232WIN_ERROR_GENERIC 1

typedef struct {
    HANDLE device;
    unsigned char isActive;
    unsigned char port[256];
    unsigned int speed;
    unsigned char parity;
    unsigned char stopBits;
    unsigned char dataBits;
    unsigned int timeout;
} T_RS232WIN_INSTANCE;

/**
 * Opens the serial port
 * @param _instance Structure that contains the information about the communication to be opened
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winOpen(T_RS232WIN_INSTANCE* _instance);

/**
 * Sends a byte array to the serial device
 * @param _bytes byte array to be written
 * @param _bytesSize size of the byte array
 * @param _bytesWritten number of bytes that was actually written
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winWrite(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesWritten);

/**
 * Reads a byte array from the serial device.
 * @param _bytes array where the read bytes will be stored
 * @param _bytesSize number of bytes expected
 * @param _bytesRead number of bytes that was actually read
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winRead(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesRead);

/**
 * Discards all characters from the output or input buffer of the connected serial device.
 * @param _clearType buffer to be cleared:
 *      RS232WIN_CLEAR_RX clears device input buffer
 *      RS232WIN_CLEAR_TX clears device output buffer
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winClear(unsigned char _clearType);

/**
 * Closes the serial port.
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winClose();

#endif // not def __linux__

#endif	/* RS232WIN_H */

