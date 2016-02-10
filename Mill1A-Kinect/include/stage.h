#ifndef __STAGE
#define __STAGE
//=============================================================================
//      
//      
//
//	File	:	Stage.h
//	Name	:	Don Uchiyama
//	Date	:	6/8/2001
//
//	Desc	: 	
//			
//			.
//
//	Defines	:	
//
//=============================================================================

//=============================================================================
//	Includes
//=============================================================================
#include "stdafx.h"
#include <vector>
//#include "Point1.h"
//#include "PathSegment.h"
//#include "Path.h"
#include "nmccom.h"
#include "sio_util.h"
#include "PathPoint.h"
#include "nmcServo.h"
#include "stagelib.h"
#include "Matrix4x4.h"
#include "Vector4.h"
#include "cmdPathIO.h"
#include "Singleton.h"
using namespace std;

class STAGELIB_API GainLimit
{
public:
	GainLimit();
	virtual ~GainLimit();
	double		dIntegralLimit; 
	double		dOutputLimit; 
	double		dCurrentLimit; 
	double		dErrorLimit; 
	double	    dServoRate;
	double		dDeadBand;
};


class STAGELIB_API Stage: public Singleton<Stage>
{

public:	
	//---- Constructors & Destructors ----
	
	Stage(  );
	Stage( const char *pComPort  );
	int			Initialize				( const char *pComPort );
	int			Initialize				( const char *pComPort, Path & path);
	Stage & operator=(Stage & stage);
	virtual ~Stage( void );	
private:
	int		    m_iError;
	int			m_iRet;
	NMCSERVO	m_NMCServo;
	vector<AxisInfo> m_Axis;
	CmdPathIO	m_IO;
	int			m_iServoModules;
	double		m_rotateAngle; // Degrees
	Matrix4x4	m_mt;

	double		m_dEncoderCountsQuad	[g_NumberOfAxis];
	double		m_dLeadScrewTreadsInch	[g_NumberOfAxis];
	double		m_dMotorGearRatio		[g_NumberOfAxis];
	double		m_dPulleyRatio			[g_NumberOfAxis];
	GainLimit   m_GainLimits			[g_NumberOfAxis];
	double      m_dVelocity				[g_NumberOfAxis];
	double      m_dAcceleration			[g_NumberOfAxis];
	
	int		    Rotate(double & x, double & y, double & z);
	int		    Rotate(vector<PathPoint> & Pathpts);

public:  // Stage Setup
	BOOL		IsInMotion			( void );
	int			GetAxis			( vector<int> & iAxis);
	int			GetGain			( eAxis nAxis,
								  int & siProportional, 
								  int & siDifferential, 
								  int & siIntegral);

	int			SetGain		    ( eAxis nAxis,
		                          int siProportional, 
						  		  int siDifferential, 
								  int siIntegral );

	int			GetGainLimits	( eAxis nAxis,
		                          GainLimit & GainLimits);

	int			SetGainLimits	( eAxis nAxis,
		                          GainLimit GainLimits);
	//---- Querying ----
	void    Rotate(double Angle);

	int		GetPos				( double & XPos,	double & YPos,		double & ZPos);
	int		GetCmdPos			( double & XPos,	double & YPos,		double & ZPos );
	int		GetPosError			( double & XError,	double & YError,	double & ZError );
	int		GetCmdVel			( double & XVel,	double & YVel,		double & ZVel );
	int		GetSpeed			( double & XVel,	double & YVel,		double & ZVel );
	int		GetCmdAccel			( double & XAccel,	double & YAccel,	double & ZAccel );
	int		GetCmdPwm			( double & XPwm,	double & YPwm,		double & ZPwm );
	int		GetAD				( double & XAD,		double & YAD,		double & ZAD );
	int		GetHome				( double & XPos,	double & YPos,		double & ZPos );
	int		GetStopPos			( double & XPos,	double & YPos,		double & ZPos );

	int		StopHere			( double lXPos, double lYPos, double lZPos  );
	int		StopMotor			( void );
	int		StopMotor			( eAxis nAxis, int nMode );
	int		EnableAmp			( void );
	int		DisableAmp			( void );

	int		Rezero				( void );
	int		ResetPos			( void );
	int		ResetRelHome		( void );

	int		MoveTo				( double lPos, int Axis, bool WithWait );
	int		MoveTo				( double lXPos, double lYPos, bool WithWait );
	int		MoveTo				( double lXPos, double lYPos, double lZPos, bool WithWait );
	int		MoveTo				( double lXPos, double lYPos, double lZPos );
	int		MoveRel				( double lXPos, double lYPos, double lZPos, bool WithWait );
	int		MoveRel				( double lXPos, double lYPos, double lZPos );
	void	EnableJogging		( bool bJogEnabled);
	
//---- Setting Options Commands ----
	int		SetEncoderCounts    ( eAxis nAxis,	double CountsPerQuadature );
	int		SetLeadScrewPitch   ( eAxis nAxis,  double ThreadPerInch );
	int		SetMotorGearRatio	( eAxis nAxis,	double GearRatio);
	int		SetPulleyRatio		( eAxis nAxis,	double PulleyRatio);
	int		GetEncoderCounts    ( eAxis nAxis,	double & CountsPerQuadature );
	int		GetLeadScrewPitch   ( eAxis nAxis,  double & ThreadPerInch );
	int		GetMotorGearRatio	( eAxis nAxis,	double & GearRatio);
	int		GetPulleyRatio		( eAxis nAxis,	double & PulleyRatio);

	int		SetPos				( double lXPos, double lYPos, double lZPos );
	int		SetPos				( eAxis nAxis,  double lPos );
	int		SetVel				( double lXVel, double lYVel, double lZVel );
	int		SetVel				( eAxis nAxis,  double lVel );
	int		SetAccel			( double lXAcc, double lYAcc, double lZAcc );
	int		SetAccel			( eAxis nAxis,  double lAcc );
	int		SetDownloadFileName	( TCHAR *pszFileName );
	
// Variables used for coordinated motion
private:
//	Path		m_Path;
	byte		m_group;			   //m_group address for coordinated controllers
	byte		m_leader;			   //m_group m_leader address for coordinated controllers
	int			m_PathFreq;		        //selected path frequency
	int			m_BufSize;   			//max num points to store in the PIC-SERVO buffer
	float		m_fScale[g_NumberOfAxis];
	float		m_PathAcceleration;
	double		m_PathSpeed;

public:	//---- Methods for Coordinated Motion ----
	int		EndCoordMotion();
	int		IsCoordMotionComplete();
	int		ExecuteCoordMotion();
	int     ExecuteCoordMotion(Path & vPath);
	int		ExecuteCoordMotionOutputPtsOnly();
	int		ClearCoordMotion();

public:	//---- Methods for Coordinated Motion ----
	int		SetGroupAddress(int group, eAxis leader);
	int		StopResetMotors();
	int		SetPathStatus();
	int     InitPathMode(Path & Path);
	int		SetPathFreq(int PathFreq);
	int		SetNumberOfPoints(int iNumberOfPoints);
	int		SetScale(double xScale, double yScale, double zScale);
	int		SetScale();
	int		SetPathAcceleration(double Acceleration);

	int		SetTangentTolerance	( double theta );
	int		ClearSegList		( double x, double y, double z );
	int		AddStart			( double x, double y, double z );
	int		AddLineSeg			( double x, double y, double z );      //end point
	int		AddArcSeg			( double x, double y, double z,        //end point
               					  double cx, double cy, double cz,     //center point
               				      double nx, double ny, double nz );   //normal
	int		AddSplineSeg( vector<double> CtrlPts, int iParts, enSplineType SplineType );    //normal
	int		SetFeedrate			( double fr);
	int		SetOrigin			( double xoffset, double yoffset, double zoffset );
	int		SetPathParams		(  );

	int		SetPathParams		( int freq, int nbuf,
								  int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
								  float xscale, float yscale, float zscale, 
								  float accel );

	int		OpenPointsOut	(string file);
	int		ClosePointsOut();
	
private:
	int		SetCoordPathMode(int pathFreq); // Sets the Cooridnated Path mode 
	int		InitPathLength( double & fPathLength );	
	int		InitPathLength( vector<double> & fPathLength );	
	int		InitCoordMotion( );	
	int		CreatePathPoints	( vector<PathPoint> & vPath, vector<double> dPathLength );
	int		AddPathPoints       ( vector<PathPoint> & vPath );
	int     AddPathPoints       ( vector<PathPoint> & vPath, int maxPointsToStore, double pathSpeed, int pathFreq );
	eServoError CheckAxis		( void );
	eServoError CheckScale		( void );
	int		ExecutePath			( );
	int		InitPath			( eAxis nAxis );
	int		StartPathMode		( int iGroupAddr, int iGroupLeader);


};

//=============================================================================
//	Inline Methods
//=============================================================================

#endif
//*** End NMCSERVO.HPP ***