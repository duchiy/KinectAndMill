//---------------------------------------------------------------------------
#ifndef _MILLCMDCTRL_H
#define _MILLCMDCTRL_H
#include "stdafx.h"
#include "millCmdCtrlLib.h"
#include "Stage.h"
#include "Path.h"
#include "DPoint.h"
#include "millCmd.h"
//---------------------------------------------------------------------------
//Defines:

using namespace std;
//using namespace vArcPath;
//using namespace vLinePath;
//Segment types:

//---------------------------------------------------------------------------
//Data types:

//template <typename T>
class MILLCMDLIB_API millCmdCtrl
{
public:
	millCmdCtrl();
	void ProcessGCode (string GCodeFile );
	void Message (string message );
	void RunGCode ( );
	void RunCommand ( int i );
	void InitRun ( );
	void Iter ( );
	int GetCommand ( );
	int Count ( );
	void GetStart ( double & X, double & Y, double & Z );
	void GetEnd ( double & X, double & Y, double & Z );
	void GetCenter ( double & X, double & Y, double & Z );
	millCmd _millCmdC;

private:
	Command *_retCmd;
};

#endif
