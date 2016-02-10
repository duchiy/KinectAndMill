//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#ifndef sio_utilH
#define sio_utilH
#endif
#include "Nmclib.h"

extern "C" {
//Serial IO function prototypes:
void NMCLIB_API STD_CALL ErrorPrinting(int f);
int NMCLIB_API STD_CALL ErrorMsgBox(char *msgstr);
int NMCLIB_API STD_CALL SimpleMsgBox(char *msgstr);
HANDLE NMCLIB_API STD_CALL SioOpen(char *name, unsigned int baudrate);
BOOL NMCLIB_API STD_CALL SioPutChars(HANDLE ComPort, char *stuff, int n);
DWORD NMCLIB_API STD_CALL SioGetChars(HANDLE ComPort, char *stuff, int n);
DWORD NMCLIB_API STD_CALL SioTest(HANDLE ComPort);
BOOL NMCLIB_API STD_CALL SioClrInbuf(HANDLE ComPort);
BOOL NMCLIB_API STD_CALL SioChangeBaud(HANDLE ComPort, unsigned int baudrate);
BOOL NMCLIB_API STD_CALL SioClose(HANDLE ComPort);

};





