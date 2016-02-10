// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the STAGELIB_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// STAGELIB_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef STAGELIB_EXPORTS
#define STAGELIB_API __declspec(dllexport)
#else
#define STAGELIB_API __declspec(dllimport)
#endif

// This class is exported from the stagelib.dll
//class STAGELIB_API Cstagelib {
//public:
//	Cstagelib(void);
	// TODO: add your methods here.
//};
//#endif


//extern STAGELIB_API int nstagelib;

//STAGELIB_API int fnstagelib(void);
