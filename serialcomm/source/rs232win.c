
#ifndef __linux__

#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include "rs232win.h"

T_RS232WIN_INSTANCE* instance;

unsigned char rs232winConfigure();
unsigned char rs232winFlush();
void getLastSystemError(int* code, char* description);

//System error variables
int sysErrCode;
unsigned char sysErrDesc[255];

/**
 * Opens the serial port
 * @param _instance Structure that contains the information about the communication to be opened
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winOpen(T_RS232WIN_INSTANCE* _instance) {    
    if (_instance == NULL) {
        return RS232WIN_ERROR_GENERIC;
    }

    instance = _instance;    
    
    instance->device = CreateFile(instance->port, (GENERIC_READ | GENERIC_WRITE), 0, 0, OPEN_EXISTING, 0, 0);

    if (instance->device != INVALID_HANDLE_VALUE) {
        if (rs232winConfigure() == RS232WIN_OK) {
            instance->isActive = 1;
            return RS232WIN_OK;
        } else {
            return RS232WIN_ERROR_GENERIC;
        }
    }

    getLastSystemError(&sysErrCode, sysErrDesc);
    return RS232WIN_ERROR_GENERIC;
}

unsigned char rs232winConfigure() {
    unsigned char dcbConfig[15];
    unsigned char i;
    DCB dcb;
    COMMTIMEOUTS cmt;

    //first, set every field of the DCB to 0 to make sure there are no invalid values
    FillMemory(&dcb, sizeof (dcb), 0);
    //set the length of the DCB
    dcb.DCBlength = sizeof (dcb);

    i = 0;
    memset(dcbConfig, 0, sizeof (dcbConfig));

    //Speed
    sprintf(dcbConfig, "%u,", instance->speed);
    i = strlen(dcbConfig);

    // Parity
    dcbConfig[i++] = instance->parity;
    dcbConfig[i++] = ',';

    // Databits
    dcbConfig[i++] = instance->dataBits;
    dcbConfig[i++] = ',';

    // Stopbits
    dcbConfig[i++] = instance->stopBits;
    dcbConfig[i++] = '\0';

    //speed,parity,data size,stop bits
    if (!BuildCommDCB(dcbConfig, &dcb)) {
        return RS232WIN_ERROR_GENERIC;
    }
    
    //TODO DTR/DSR flow control
    dcb.fDsrSensitivity = TRUE;
    dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;
    dcb.fOutxDsrFlow = TRUE;
    
    //set the state of fileHandle to be dcb returns a boolean indicating success or failure
    if (!SetCommState(instance->device, &dcb)) {
        return RS232WIN_ERROR_GENERIC;
    }
    //set the buffers to be size 2048 of fileHandle. Also returns a boolean indicating success or failure
    if (!SetupComm(instance->device, 2048, 2048)) {
        return RS232WIN_ERROR_GENERIC;
    }

    /*The maximum time allowed to elapse before the arrival of the next byte on 
    the communications line, in milliseconds. If the interval between the arrival 
    of any two bytes exceeds this amount, the ReadFile operation is completed and 
    any buffered data is returned. A value of zero indicates that interval 
    time-outs are not used.*/
    cmt.ReadIntervalTimeout = 200;

    /*The multiplier used to calculate the total time-out period for read operations, 
    in milliseconds. For each read operation, this value is multiplied by the 
    requested number of bytes to be read.*/
    cmt.ReadTotalTimeoutMultiplier = 500;

    /*
     * A constant used to calculate the total time-out period for read operations, 
     * in milliseconds. For each read operation, this value is added to the product 
     * of the ReadTotalTimeoutMultiplier member and the requested number of bytes.
     * A value of zero for both the ReadTotalTimeoutMultiplier and 
     * ReadTotalTimeoutConstant members indicates that total time-outs are not 
     * used for read operations.
     */
    cmt.ReadTotalTimeoutConstant = 3000;

    /*The multiplier used to calculate the total time-out period for write 
     * operations, in milliseconds. For each write operation, this value is 
     * multiplied by the number of bytes to be written.*/
    cmt.WriteTotalTimeoutMultiplier = 500;
    
    /*A constant used to calculate the total time-out period for write operations, 
    in milliseconds. For each write operation, this value is added to the product 
    of the WriteTotalTimeoutMultiplier member and the number of bytes to be written.
    A value of zero for both the WriteTotalTimeoutMultiplier and 
    WriteTotalTimeoutConstant members indicates that total time-outs are not 
    used for write operations.*/
    cmt.WriteTotalTimeoutConstant = 3000;

    //set the timeouts of fileHandle to be what is contained in cmt returns boolean success or failure
    if (!SetCommTimeouts(instance->device, &cmt)) {
        return RS232WIN_ERROR_GENERIC;
    }

    return RS232WIN_OK;
}

/**
 * Sends a byte array to the serial device
 * @param _bytes byte array to be written
 * @param _bytesSize size of the byte array
 * @param _bytesWritten number of bytes that was actually written
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winWrite(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesWritten) {
    DWORD write = -1;
    
    if (WriteFile(instance->device, _bytes, _bytesSize, &write, NULL)) {
        *_bytesWritten = (unsigned int) write;
        return RS232WIN_OK;
    } else {
        return RS232WIN_ERROR_GENERIC;
    }
}

/**
 * Reads a byte array from the serial device.
 * @param _bytes array where the read bytes will be stored
 * @param _bytesSize number of bytes expected
 * @param _bytesRead number of bytes that was actually read
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winRead(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesRead) 
{
    DWORD read = -1;
    
    if (ReadFile(instance->device, _bytes, _bytesSize, &read, NULL)) {
        *_bytesRead = (unsigned int) read;
        return RS232WIN_OK;
    } else {
        return RS232WIN_ERROR_GENERIC;
    }
}

/**
 * Closes the serial port.
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winClose() {
    
    //Close the fileHandle, thus releasing the device.
    if (instance->isActive) {
        if (!CloseHandle(instance->device)) {
            return RS232WIN_ERROR_GENERIC;
        }
        instance->isActive = 0;
    }
    
    return RS232WIN_OK;
}

/**
 * Discards all characters from the output or input buffer of the connected serial device.
 * @param _clearType buffer to be cleared:
 *      RS232WIN_CLEAR_RX clears device input buffer
 *      RS232WIN_CLEAR_TX clears device output buffer
 * @return RS232WIN_OK if success. RS232WIN_ERROR_* if error.
 */
unsigned char rs232winClear(unsigned char _clearType)
{
    DWORD purgeFlags = 0;
    
    switch(_clearType)
    {
        case RS232WIN_CLEAR_RX:
            purgeFlags = PURGE_RXCLEAR;
            break;
        case RS232WIN_CLEAR_TX:
            purgeFlags = PURGE_TXCLEAR;
            break;
        default:
            return RS232WIN_ERROR_GENERIC;
            break;
    }
    
    if(PurgeComm(instance->device, purgeFlags))
    {
        return RS232WIN_OK;
    }
    else
    {
        return RS232WIN_ERROR_GENERIC;
    }
}

unsigned char rs232winFlush() {
    if (PurgeComm(instance->device, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
        return RS232WIN_OK;
    } else {
        return RS232WIN_ERROR_GENERIC;
    }
}

void getLastSystemError(int* code, char* description)
{ 
    // Retrieve the system error message for the last-error code
    LPVOID lpMsgBuf;
    DWORD dw = GetLastError(); 

    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        dw,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL );
    
    *code = (int) dw;
    strcpy(description, (unsigned char*)lpMsgBuf);
    
    LocalFree(lpMsgBuf);
    return;
}

#endif // not def __linux__
