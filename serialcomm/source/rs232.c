
#include <stdio.h>
#include <stdlib.h>
#include "rs232.h"

#ifdef __linux__
#include "rs232lnx.h"
#else
#include "rs232win.h"
#endif

unsigned char rs232Open(T_RS232_INSTANCE* _instance) {
#ifdef __linux__
    return rs232lnxOpen((T_RS232LNX_INSTANCE*) _instance);
#else
    return rs232winOpen((T_RS232WIN_INSTANCE*) _instance);
#endif
}

unsigned char rs232Write(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesWritten) {
#ifdef __linux__
    return rs232lnxWrite(_bytes, _bytesSize, _bytesWritten);
#else
    return rs232winWrite(_bytes, _bytesSize, _bytesWritten);
#endif
}

unsigned char rs232Read(unsigned char* _bytes, unsigned int _bytesSize, unsigned int* _bytesRead) {
#ifdef __linux__
    return rs232lnxRead(_bytes, _bytesSize, _bytesRead);
#else
    return rs232winRead(_bytes, _bytesSize, _bytesRead);
#endif
}

unsigned char rs232clear(unsigned char _clearType)
{
   #ifdef __linux__
    return rs232lnxClear(_clearType);
#else
    return rs232winClear(_clearType);
#endif 
}

unsigned char rs232Close() {
#ifdef __linux__
    return rs232lnxClose();
#else
    return rs232winClose();
#endif
}