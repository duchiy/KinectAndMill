#ifndef __NMCSERVO
#define __NMCSERVO
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
#include <vector>
//#include "Point1.h"
//#include "PathSegment.h"
#include "picservo.h"
#include "Path.h"
#include "Stagelib.h"
using namespace std;

//=============================================================================
//	Conditional Compilation Defines
//=============================================================================


//=============================================================================
//	Constants
//=============================================================================
const int FILENAME_LEN =255;
const int g_NumberOfAxis = 6;

//=============================================================================
//	Typedefs
//-----------------------------------------------------------------------------

//=============================================================================
//	Class Description
//=============================================================================

//class NMCSERVO : public MOTION
enum eAxis				{ eXAxis=0, eYAxis=1, eZAxis=2  }; 
enum ePathSegmentError	{ eSegNotTangent=-1, eSegListFull=-2, eSegInvalidArc, eSegNotAdded  }; 
enum ePathError			{ eCommError=-1, ePathNotComplete=-2, ePathIncompleteMotorStat=-3, ePathPointNotRead=-4  }; 
enum ePathReturn		{ ePathDownloadComplete=0 };
enum eServoError	    { eInvalidAxis=-1, eInvalidScale=-2, eInvalidAcceleration=-3, 
                          eInvalidPathFreq=-4, eInvalidBufferSize=-5, eFeedRateNotSet=-6};

class AxisInfo
{
public:
	AxisInfo();
	~AxisInfo();
	byte	byAxis;
	byte	byStopMode;
	byte	byIOMode;
	byte	byServoMode;
	byte	byHomingMode;
	byte	byStatusMode;
};
class IOInfo
{
public:
	IOInfo();
	~IOInfo();
	byte byIO;
};

class NMCGainLimit
{
public:
	NMCGainLimit();
	virtual ~NMCGainLimit();
	short int	siIntegralLimit; 
	byte		byOutputLimit	 ; 
	byte		byCurrentLimit ; 
	short int	siErrorLimit	 ; 
	byte        byServoRate	 ;
	byte	    byDeadBand	 ;
};

class STAGELIB_API NMCSERVO
{
private:
	vector<AxisInfo>	m_Axis;

protected:


	int			m_iModules;
	int			m_iServoModules;
	int			m_iIOModules;
	IOInfo		m_IO[MAXNUMMOD-g_NumberOfAxis];
	WORD		m_wStatusAddress;
	TCHAR		m_szFileName[ FILENAME_LEN ];

	bool		m_bJogEnabled;
	
	NMCGainLimit m_NMCGainLimits[g_NumberOfAxis];

	long        m_lVelocity		[g_NumberOfAxis];
	long        m_lAcceleration [g_NumberOfAxis];

	double		m_dPathLength;


public:	
	//---- Constructors & Destructors ----
	
	NMCSERVO(  );
	virtual ~NMCSERVO( void );
	
	int			Initialize				( const char *pComPort );
	int			GetAxisInfo				( vector <AxisInfo> & AxisInfo );
	

public:	 // New Queries
	//---- Querying ----
	int			 GetMaxAccel			( eAxis nAxis, double & dAcceleration );
	int			 GetMaxSpeed			( eAxis nAxis, double & dSpeed );
	int			 GetMaxRezeroTimeout	( double & dTimeOut );
	int			 GetMaxPos				( eAxis nAxis, double & dPosition );
	int			 GetMinPos				( eAxis nAxis, double & dPosition );
	int			 GetRezeroStatus		( void );
	BOOL		 IsInMotion				( eAxis nAxis );
	int			 IsTrackingError		( void );

	long		 GetCmdPosition		( eAxis nAxis );
	long		 GetCmdSpeed		( eAxis nAxis );
	long		 GetCmdAccel		( eAxis nAxis );
	unsigned int GetCmdPwm			( eAxis nAxis );

	long		 GetPosition		( eAxis nAxis );
	long		 GetSpeed			( eAxis nAxis );
	long         GetAccel			( eAxis nAxis );
	int			 GetPositionError	( eAxis nAxis );
	long		 GetAD				( eAxis nAxis );
	long		 GetHome			( eAxis nAxis );

	int			 GetAux				( eAxis nAxis );
	long		 GetStopPosition	( eAxis nAxis );

	unsigned int GetMoveCtrl		( eAxis nAxis );
	unsigned int GetStopCtrl		( eAxis nAxis );
	unsigned int GetHomeCtrl		( eAxis nAxis );
	unsigned int GetIOCtrl			( eAxis nAxis );

	int			 GetGain			( eAxis nAxis,
									  int & siProportional, 
									  int & siDifferential, 
									  int & siIntegral);

	int			 SetGain		    ( eAxis nAxis,
		                              int siProportional, 
						  			  int siDifferential, 
									  int siIntegral );

	int			 GetGainLimits		( eAxis nAxis,
									  int & iIntegralLimit, 
									  int & iOutputLimit, 
									  int & iCurrentLimit, 
									  int & iErrorLimit,
									  int & iServoRate,
								  	  int & iDeadBand);

	int			 SetGainLimits		( eAxis nAxis,
									  int iIntegralLimit, 
									  int iOutputLimit, 
									  int iCurrentLimit, 
									  int iErrorLimit,
									  int iServoRate,
								  	  int iDeadBand);
public:
// Initialization Commands 

	int			ResetPosition		( eAxis nAxis );
	int			ResetRelHome		( eAxis nAxis );
	int			SetPosition			( eAxis nAxis, long lPosition );
	int			ClearBits			( eAxis nAxis );

	int			StopMotor			( eAxis nAxis );
	int			StopMotor			( eAxis nAxis, int nMode );
	int			StopHere			( eAxis nAxis, int nMode, long lPosition);
	int			SetIOCtrl			( eAxis nAxis, int iMode );

	int			StartMove			( int iGroupAddr, int iGroupLeader);
	int			SetHoming			( eAxis nAxis, int iMode);
	int			HardReset			( eAxis nAxis, int iMode);

	
//---- Action Commands ----	
	int			Rezero				( eAxis nAxis );
	int			MoveTo				( long lPos,  eAxis nAxis  );
	int			MoveRel				( long lPos,  eAxis nAxis  );

	
//---- Setting Options Commands ----
	int			SetSpeed			( eAxis nAxis,long lSpeed );
	int			SetAccel			( eAxis nAxis,long lAcceleration);
	int			SetDownloadFileName	( TCHAR *pszFileName );
	
protected:
	//---- Private Methods ----
	BOOL		m_bError     ;
	int			m_iError     ;
	int			m_iRet;

private:	
	int			SetDefaultIOMode    ();
	int			SetDefaultServoMode ();
	int			SetDefaultHomeMode  ();
	int			SetDefaultStatusMode();
	int			SetDefaultStopMode  ();

public:
	int			EnableAmp    (eAxis nAxis);
	int			DisableAmp   (eAxis nAxis);
	int			SetStopMode  (eAxis nAxis, byte byMode);
	int			SetIOMode    (eAxis nAxis, byte byMode);
	int			SetServoMode (eAxis nAxis, byte byMode);
	int			SetHomeMode  (eAxis nAxis, byte byMode);
	int			SetStatusMode(eAxis nAxis, byte byMode);

};

//=============================================================================
//	Inline Methods
//=============================================================================

#endif
//*** End NMCSERVO.HPP ***