//---------------------------------------------------------------------------
#ifndef _CMDIO_H
#define _CMDIO_H
#include "stdafx.h"
#include "Pathlib.h"
//---------------------------------------------------------------------------
//Defines:
#include <math.h>
#include "vPathSegment.h"
#include "vLinePath.h"
#include "vArcPath.h"
#include "vSplinePath.h"
#include <EnumTypes.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <Spline.h>
#include <SplineFactory.h>
#include "stagelib.h"
using namespace std;
//using namespace vArcPath;
//using namespace vLinePath;
//Segment types:

//template <typename T>
class vPathSegment;

class CmdPathIO
{
public:
	CmdPathIO();
	~CmdPathIO( void );


private:	
	ofstream m_outfile;
	ifstream m_infile;

public:
	int		ReadPathSegment();
	int	    SavePathToFile( );
	ofstream & GetOutFile();
	ifstream & GetInFile();

	int		OpenOutFile(string pathfile);
	int		OpenInFile(string pathfile);

	int		CloseOutFile();
	int		CloseInFile();
	int		WriteCmd(enCommand enCmd);
	int		WriteXYZ (double x, double y, double z);
	int		ReadXYZ	 (double & x, double & y, double & z);
	int		ReadCmd	 (enCommand & enCmd);

};

#endif
