
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the NMCLIB_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// NMCLIB_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#include <windows.h>
#ifdef NMCLIB_EXPORTS
#define NMCLIB_API __declspec(dllexport) 
#else
#define NMCLIB_API __declspec(dllimport) 
#endif

#ifndef STD_CALL
#define STD_CALL __stdcall 
#endif

// This class is exported from the nmclib.dll
//class NMCLIB_API CNmclib {
//public:
//	CNmclib(void);
	// TODO: add your methods here.
//};

#define  mod _mod
#define  nummod _nummod
#define  ComPort _ComPort
#define  SioError _SioError
#define  IOBusy _IOBusy


