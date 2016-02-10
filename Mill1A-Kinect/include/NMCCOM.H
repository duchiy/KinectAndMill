//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#ifndef nmccomH
#define nmccomH
#include <Nmclib.h>
#include <NMCCOMTYPES.h>
/*
typedef unsigned char byte;

typedef struct _NMCMOD {
	byte	modtype;		//module type
    byte	modver;			//module version number
	byte	statusitems;	//definition of items to be returned
	byte	stat;  			//status byte
    byte	groupaddr;		//current group address
    BOOL	groupleader;	//TRUE if group leader
    void *	p;				//pointer to specific module's data structure
    } NMCMOD;

#define MAXSIOERROR 2

//Define PIC baud rate divisors
#define	PB19200		64
#define	PB57600		21
#define	PB115200	10
#define	PB230400	5

//Module type definitions:
#define	SERVOMODTYPE	0
#define	ADCMODTYPE		1
#define	IOMODTYPE		2
#define	STEPMODTYPE		3
//The following must be created for each new module type:
//		data structure XXXMOD
//		Initializer function NewXXXMod
//		Status reading function GetXXXStat
//		NMCInit and SendNmcCmd must be modified to include calls
//			to the two functions above

#define CKSUM_ERROR		0x02	//Checksum error bit in status byte


#define MAXNUMMOD	33
*/
//Function prototypes:
extern "C" {

//Initialization and shutdown
int NMCLIB_API STD_CALL NmcInit(char *portname, unsigned int baudrate);
void NMCLIB_API STD_CALL InitVars(void);
BOOL NMCLIB_API STD_CALL NmcSendCmd(byte addr, byte cmd, char *datastr, byte n, byte stataddr);
void NMCLIB_API STD_CALL FixSioError(void);
void NMCLIB_API STD_CALL NmcShutdown(void);

//Module type independant commands (supported by all module types)
BOOL NMCLIB_API STD_CALL NmcSetGroupAddr(byte addr, byte groupaddr, BOOL leader);
BOOL NMCLIB_API STD_CALL NmcDefineStatus(byte addr, byte statusitems);
BOOL NMCLIB_API STD_CALL NmcReadStatus(byte addr, byte statusitems);
BOOL NMCLIB_API STD_CALL NmcSynchOutput(byte groupaddr, byte leaderaddr);
BOOL NMCLIB_API STD_CALL NmcChangeBaud(byte groupaddr, unsigned int baudrate);
BOOL NMCLIB_API STD_CALL NmcSynchInput(byte groupaddr, byte leaderaddr);
BOOL NMCLIB_API STD_CALL NmcNoOp(byte addr);
BOOL NMCLIB_API STD_CALL NmcHardReset(byte addr);

//Retrieve module type independant data from a module's data structure
byte NMCLIB_API STD_CALL NmcGetStat(byte addr);
byte NMCLIB_API STD_CALL NmcGetStatItems(byte addr);
byte NMCLIB_API STD_CALL NmcGetModType(byte addr);
byte NMCLIB_API STD_CALL NmcGetModVer(byte addr);
byte NMCLIB_API STD_CALL NmcGetGroupAddr(byte addr);
BOOL NMCLIB_API STD_CALL NmcGroupLeader(byte addr);
};
#endif



