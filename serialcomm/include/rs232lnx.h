#ifndef RS232LNX_H
#define	RS232LNX_H

#ifdef __linux__

#define T_RS232LNX_DEVICE int

#define RS232LNX_SPEED_9600 9600
#define RS232LNX_SPEED_115200 115200

#define RS232LNX_PARITY_NONE 0
#define RS232LNX_PARITY_ODD 1
#define RS232LNX_PARITY_EVEN 2

#define RS232LNX_STOPBITS_1 1
#define RS232LNX_STOPBITS_2 2

#define RS232LNX_DATABITS_5 5
#define RS232LNX_DATABITS_6 6
#define RS232LNX_DATABITS_7 7
#define RS232LNX_DATABITS_8 8

#define RS232LNX_OK 0
#define RS232LNX_ERROR_GENERIC 1

typedef struct {
    T_RS232LNX_DEVICE device;
    unsigned char isActive;
    unsigned char port[256];
    unsigned int speed;
    unsigned char parity;
    unsigned char stopBits;
    unsigned char dataBits;
    unsigned int timeout;
} T_RS232LNX_INSTANCE;

/**
 * Opens the serial port
 * @param _instance Structure that contains the information about the communication to be openned
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxOpen(T_RS232LNX_INSTANCE* _instance);

/**
 * Sends a byte array to the serial device
 * @param _bytes byte array to be written
 * @param _bytesSize size of the byte array
 * @param _bytesWritten number of bytes that was actually written
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxWrite(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesWritten);

/**
 * Reads a byte array from the serial device.
 * @param _bytes array where the read bytes will be stored
 * @param _bytesSize number of bytes expected
 * @param _bytesRead number of bytes that was actually read
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxRead(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesRead);

/**
 * Discards all characters from the output or input buffer of the connected serial device.
 * @param _clearType buffer to be cleared:
 *      RS232WIN_CLEAR_RX clears device input buffer
 *      RS232WIN_CLEAR_TX clears device output buffer
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232lnxClear(unsigned char _clearType);

/**
 * Closes the serial port.
 * @return RS232LNX_OK if success. RS232LNX_ERROR_* if error.
 */
unsigned char rs232lnxClose();

#endif // __linux__

#endif	/* RS232LNX_H */

