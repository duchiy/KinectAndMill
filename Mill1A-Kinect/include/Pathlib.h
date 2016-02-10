
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the PATHLIB_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// PATHLIB_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef PATHLIB_EXPORTS
#define PATHLIB_API __declspec(dllexport)
#else
#define PATHLIB_API __declspec(dllimport)
#endif

#ifndef STD_CALL
#define STD_CALL __stdcall 
#endif

// This class is exported from the Pathlib.dll
//class PATHLIB_API CPathlib {
//public:
//	CPathlib(void);
	// TODO: add your methods here.
//};

//extern PATHLIB_API int nPathlib;

//PATHLIB_API int fnPathlib(void);

