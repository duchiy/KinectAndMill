//---------------------------------------------------------------------------
#ifndef _PATHLIB_H
#define _PATHLIB_H
#include "stdafx.h"
#include "Pathlib.h"
//---------------------------------------------------------------------------
//Defines:
#include <math.h>
#include "nmccom.h"
#include "sio_util.h"
#include "vPathSegment.h"
#include "vLinePath.h"
#include "vArcPath.h"
#include "vSplinePath.h"
#include "vFrame.h"
#include "PathPoint.h"
#include <EnumTypes.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <Spline.h>
#include <SplineFactory.h>
#include "Singleton.h"

using namespace std;
//using namespace vArcPath;
//using namespace vLinePath;
//Segment types:
#define LINE 0
#define ARC 1

#define MAXSEG 1000     //Maximum number of segments
#define PI 3.14159
#define TWOPI 6.28319
#define DTOR 0.017453

//Valuse for tangent tolerance
#define TAN_1DEGREE 0.99985
#define TAN_3DEGREE 0.99863
#define TAN_5DEGREE 0.99619
#define TAN_10DEGREE 0.98481
#define TAN_20DEGREE 0.93969
#define TAN_45DEGREE 0.70711

#define ONLINE 1
//---------------------------------------------------------------------------
//Data types:
enum enSegMessage {enEndOfPath=-2, enNoOrigin, enNormalPathRet=0, enInvalidSeg, enEndOfSeg, enLastPointOfPath=1};

//template <typename T>
class PATHLIB_API Path : public Singleton<Path>
{
private:
	list<vPathSegment*> m_vsegList;
	list<vPathSegment*>::iterator  m_vsegListItr;

	double	m_TangentTol;           //minimum cos(th) for angle between tangents
	vFrame	m_vCurrArcFrame1;		//coordinate fram for the current arc
	int		m_PathFreq;		        //selected path frequency
	int		m_BufSize;   			//max num points to store in the PIC-SERVO buffer

	float	m_CmdMaxVel;			//max. commanded velocity
	float   m_CmdAcc;		        //commanded acceleration

	byte	m_x;  				    //axes addresses
	byte	m_y;
	byte	m_z;
	byte	m_group;			   //m_group address for coordinated controllers
	byte	m_leader;			   //m_group m_leader address for coordinated controllers
//	int		m_FinalDecel;		   //flag for when final deceleration has started
	double	m_XOffset, m_YOffset, m_ZOffset;  		//origin offset
	double	m_UnitsToCountsX, m_UnitsToCountsY, m_UnitsToCountsZ;	//Units TO Counts conversion factors
	double	m_Tol;				   //small distance m_Tol use for near-zero comparisons
	long int m_points;
	ofstream m_fpoints;
	SplineFactory m_SplineFactory;

private:
	double mag	 ( vector<double> Pt );
	double dot	 ( vector<double> Pt1, vector<double> Pt2 );
	int normalize( vector<double> & Pt );
	vector<double> scale    ( double Scale, vector<double>  Pt );
	vector<double> cross	( vector<double> Pt1, vector<double> Pt2 );      // x=x cross y
	vector<double> subtract ( vector<double> Pt1, vector<double> Pt2 );  // z=x-y
	vector<double> fvmult(vFrame F, vector<double> Pt );
	int		GetTanVectP(vPathSegment *s, vector<double> & Pt, bool bEnd);
	int		GetPathPointsA(vector<PathPoint> & vPath, double CmdPath);
	int		GetPathPoint(bool & bNewSegment, double SegPathLength, vector<PathPoint> & rPt );
	void	GetArcFrameP(vArc *seg, vFrame *F);
	vector<double>   GetPointOnLine(vLine Line, double fLength);
	vector<double>   GetPointOnArc (vArc Arc, double fLength);
	vector<double>   GetPointOnSpline( vSpline Spline, double fLength );
	vector<double>   GetPointOnSplineA( vSpline Spline, double fLength );


public:
	Path();
	~Path();
	int		OpenPointsOut(string file);
	int		ClosePointsOut();
	int		SetAxis( vector<int> mAxis );
	int		GetCmdPathSpeed(double & Speed);
	int		GetPathFreq(int & pathFreq);
	int		GetPathFreq();
	int		GetMaxPointsToStore(int & maxPoints);
	int     GetScale(double & xscale, double & yscale, double & zscale);
	void	SetTangentTolerance(float theta);

	void	ClearSegListA(double x, double y, double z);
	int		AddStartA(double x, double y, double z);
	int		AddLineSegA(double x, double y, double z);
	int		AddArcSegA( double x, double y, double z,        //end point
               			double cx, double cy, double cz,     //center point
               			double nx, double ny, double nz );    //normal
	int		AddSplineSegP( vector<double> CtrlPts, int iParts, enSplineType SplineType );    //normal

	void	SetFeedrate(double fr);
	void	SetOrigin(double xoffset, double yoffset, double zoffset);
	int		SetPathParams(int freq, int nbuf,
						   double xscale, double yscale, double zscale,
						   double accel );

	double	InitPathLength(double & CmdPath );
	int		InitPathLength(vector<double> & CmdPath );
	int		AddPathPointsV(vector<PathPoint> & vPath);
	int		CreatePathPointsP(vector<PathPoint> & vPath, vector<double> fPathLength);
	int		ReadPathSegment();
	int	    SavePathToFile( );
	int	    SavePathToFile(char *lpszFileName, vector<PathPoint> & vPath );
	int	    SavePathToFile(vector<PathPoint> & vPath );
	
	int	    ClearList();
	Path& operator=(Path & path) {
       return path;
    }
};

#endif
