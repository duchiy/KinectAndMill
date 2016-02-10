#ifndef __VSEGMENT_H__
#define __VSEGMENT_H__
#include "stdafx.h"
//--------------------------------------------------------------------------------
//	Class Name:  PathSegment
//	
//	Description:
//	This class is a base class inherited by Line, and Arc.  This class is used
//  in a link list for Motion Path.  A Motion Path is made up of Line, and Arc 
//	segment.
//
//
//	
//	History:
//
//
//
//
//
//
//
//
//--------------------------------------------------------------------------------

typedef float fPt[3];     //floating point 3x1 vector
#include <vector>

#include <Point.h>
#include <EnumTypes.h>
#include <iostream>
#include <fstream>
#include "Pathlib.h"

using namespace std;

#ifndef __SEGMENT_H__
enum enType {enOrigin, enLine, enArc, enSpline};
#endif
class PATHLIB_API vPathSegment
{
public:
	vPathSegment();
	virtual ~vPathSegment();
	enType m_segType;
	int SetStart	(float fX, float fY, float fZ);
	int GetStart	(float & fX, float & fY, float & fZ);
	int SetEnd		(float fX, float fY, float fZ);
	int SetEnd		(fPt fEnd);
	int SetEnd		(Point fEnd);
	int SetEnd		(vector<double> fEnd);
	int GetEnd		(float & fX, float & fY, float & fZ);
	int SetLength	(float fLength);
	vPathSegment	&	operator=(vPathSegment & rPathSeg);
	vPathSegment	&	operator=(vPathSegment * pPathSeg);
public:
    float	m_fLength;		 //PathSegment length
	vector<double>   m_Start;
	vector<double>   m_End;
	virtual void Draw() {};
	friend ofstream & operator << (ofstream & out, const vPathSegment & path);
	friend ifstream & operator >> (ifstream & in, vPathSegment & path);
};

#endif
