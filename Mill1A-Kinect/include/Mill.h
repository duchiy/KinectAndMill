#ifndef __MILL__
#define __MILL__
//=============================================================================
//      
//      
//
//	File	:	NMCSERVO.h
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
#include "StdAfx.h"
#include "nmcServo.h"
#include "Path.h"
using namespace std;

//=============================================================================
//	Conditional Compilation Defines
//=============================================================================


//=============================================================================
//	Constants
//=============================================================================


//=============================================================================
//	Typedefs
//-----------------------------------------------------------------------------

//=============================================================================
//	Class Description
//=============================================================================

class Mill
{

private:
	NMCSERVO m_Stage;
	Path	 m_Path;

public:	
	//---- Constructors & Destructors ----
	
	Mill(  );
	virtual ~Mill( void );
	
public:	 
//---- Scaling ----
	//---- Scaling Option Queries ----
	int			GetEncoderCountsQuad	( int nAxis, double & dEncoderCounts );
	int			GetLeadScrewThreadsInch( int nAxis, double & dThreadsInch );
	int			GetMotorGearRatio		( int nAxis, double & dGearRatio );

	//---- Scaling Options Commands ----
	int			SetEncoderCountsQuad( int nAxis,	double dEncoderCounts );
	int			SetLeadScrewThreadsInch( int nAxis, double dThreadsInch );
	int			GetMotorGearRatio	( int nAxis,	double dGearRatio);


//--- Gain Information
	//--- Querying Gain Information
	int			 GetGain			( int nAxis,
									  int & siProportional, 
									  int & siDifferential, 
									  int & siIntegral);

	int			 GetGainLimits		( int & iIntegralLimit, 
									  int & iOutputLimit, 
									  int & iCurrentLimit, 
									  int & iErrorLimit,
									  int & iServoRate,
								  	  int & iDeadBand);
	//--- Setting Gain Information
	int			SetGain				( int nAxis,
		                              int siProportional, 
						  			  int siDifferential, 
									  int siIntegral );

	int			SetGainLimits		( int iIntegralLimit, 
									  int iOutputLimit, 
									  int iCurrentLimit, 
									  int iErrorLimit,
									  int iServoRate,
								  	  int iDeadBand);
//---- Speed and Acceleration
	//---- Querying Speed and Acceleration
	long		GetSpeed			( int nAxis );
	long        GetAccel			( int nAxis );

	//---- Setting Speed and Acceleration
	int			SetSpeed			( long lSpeed );
	int			SetAccel			( long lAcceleration);


//-----Motion and Position
	//---- Querying Motion and Position
	int			 GetPosition			( long & rlXPos, long & rlYPos, long & rlZPos);
	BOOL		 IsInMotion				( void );
	int			 IsTrackingError		( void );

	//---- Setting Motion and Position
	int			SetPosition			( long lXPos, long lYPos, long lZPos );
	int			ResetPosition		( void );
	int			ResetRelHome		( void );

	int			MoveTo				( long lXPos, long lYPos, long lZPos );
	int			MoveRel				( long lXPos, long lYPos, long lZPos  );
	int			StopHere			( long lXPos, long lYPos, long lZPos  );


public:

	int			SetDownloadFileName	( TCHAR *pszFileName );
	
public:	//---- Methods for Coordinated Motion ----
	int			SetGroupAddress(int group, int leader);
	int			StopResetMotors();
	int			SetPathStatus();
	int			SetPathFreq(int PathFreq);
	int			SetNumberOfPoints(int iNumberOfPoints);
	int			SetAxisAddress(int x, int y, int z);
	int			SetScale(double xScale, double yScale, double zScale);
	int			SetPathAcceleration(double Acceleration);
	int			SetCoordPathMode(); // Sets the Cooridnated Path mode 

	int			ClearSegList		( double x, double y, double z );
	int			AddLineSeg			( double x, double y, double z );      //end point
	int			AddArcSeg			( double x, double y, double z,        //end point
               						  double cx, double cy, double cz,     //center point
               						  double nx, double ny, double nz );   //normal
	int			SetOrigin			( double xoffset, double yoffset, double zoffset );
	int			SetFeedrate			( double fr);
	int			SetTangentTolerance	( double theta );
	int			SetPathParams		(  );

	int			IsCoordMotionComplete();
	int			ExecuteCoordMotion();
	int			ClearCoordMotion();


private:
	int			InitPathLength( );	
	int			InitCoordMotion( );	
	int			CreatePathPoints	( vector<PathPoint> & vPath );
	int			AddPathPoints       ( vector<PathPoint> & vPath );
	int			StartPathMode		( int iGroupAddr, int iGroupLeader);

	
private:
	//---- Private Methods ----

};

//=============================================================================
//	Inline Methods
//=============================================================================

#endif
//*** End NMCSERVO.HPP ***