#ifndef __MOTIONH
#define __MOTIONH
//=============================================================================
//
//	File	:	motion.h
//	Name	:	Don Uchiyama
//	Date	:	6/8/2001
//
//	Desc	: 	
//
//=============================================================================

//=============================================================================
//	Includes
//=============================================================================

//=============================================================================
//	Conditional Compilation Defines
//=============================================================================

//=============================================================================
//	Constants
//=============================================================================
const DWORD MOTION_ERR	= 0x00000001L;
const DWORD MOTION_ERR_COMM_FAILURE	= 0x00000002L;
const DWORD MOTION_ERR_DEVICE_BUSY = 0x00000008L;
const DWORD MOTION_ERR_INVALID_PARAM = 0x00000010L;
const DWORD MOTION_ERR_REZERO	= 0x00000020L;
const DWORD MOTION_ERR_REZERO_WAIT = 0x00000040L;
const DWORD MOTION_ERR_IN_MOTION = 0x00000400L;
const DWORD MOTION_ERR_AT_LIMIT = 0x00000800L;
const DWORD MOTION_ERR_MEM_ALLOC = 0x00002000L;
const DWORD MOTION_ERR_FUNCTION_UNAVAIL = 0x00004000L;
const DWORD MOTION_ERR_INIT_FAILED = 0x00008000L;

const DWORD MOTION_ERR_ALL = 0xFFFFFFFFL;
const DWORD MOTION_ERR_CRITICAL = MOTION_ERR_INIT_FAILED;

const WORD MOTION_MAX_AXES = 8;

const WORD MAX_MOTION_PROFILE = 255; //maximum number of motion profile for spiral tool.

//=============================================================================
//	Typedefs
//=============================================================================
	//---------------------------------------
	//  MOTION method pointer types.
	//---------------------------------------
class MOTION;

typedef double (MOTION::*PFN_MOTIONDF)( void );
typedef bool   (MOTION::*PFN_MOTIONB)( void );
typedef DWORD  (MOTION::*PFN_MOTIONDW)( void );
typedef DWORD  (MOTION::*PFN_MOTIONDWDF)( double );

typedef double (MOTION::*PFN_MOTIONDFAXIS)( int );
typedef bool   (MOTION::*PFN_MOTIONBAXIS)( int );
typedef DWORD  (MOTION::*PFN_MOTIONDWAXIS)( int );
typedef DWORD  (MOTION::*PFN_MOTIONDWDFAXIS)( double, int );

//=============================================================================
//	Class Description
//=============================================================================
class MOTION 
{

protected:
	int m_nMaxAxisCount;
	DWORD m_dwAddress;
	DWORD m_fdwError;
	DWORD m_fdwStatus;
	bool m_bJoystickEnabled;
   static bool ms_bMotionDisabled;
	double m_adfHomePos[MOTION_MAX_AXES];

	//---- Methods ----
	DWORD	clearErrorBits( DWORD dwErrCode );
	DWORD setErrorBits( DWORD dwErrCode );

	bool isAxisValid( int nIdx );
	bool cleanUp( void );
	virtual bool move( double dfPos, int nAxis, bool bRelative )		= 0;
	virtual bool moveXY( double dfXPos, double dfYPos, bool bRelative )     = 0;

public:
	//---- Constructors & Destructors ----
	MOTION( WORD wMaxAxisCount = 1 );
	virtual ~MOTION( void );

	virtual DWORD Initialize( WORD wIOAddress );

	//---- Querying ----
	virtual DWORD  GetError( void );
	virtual double GetJSAccelDecel( int nAxis ) = 0;
	virtual bool   GetJSMovedFlag( );
	virtual double GetMotorCountsPerMM( int nAxis )     = 0;
	virtual double GetPositionCountsPerMM( int nAxis )  = 0;
	virtual double GetMaxAccel( int nAxis )             = 0;
	virtual double GetMaxSpeed( int nAxis )             = 0;
	virtual double GetMaxPos( int nAxis ) = 0;
	virtual double GetMinPos( int nAxis ) = 0;
	virtual double GetSpeed( int nAxis )                = 0;
	virtual double GetAccel( int nAxis )                = 0;
	virtual double GetPosition( int nAxis )             = 0;
	virtual double GetCommandedPosition( int nAxis )    = 0;
	virtual DWORD  GetRezeroStatus( void )              = 0;
	virtual double GetHome( int nAxis );
	virtual bool   GetLimits( double *pdfBWLimit, double *pdfFWLimit, int nAxis )   = 0;
	virtual double GetMaxRezeroTimeout( void )          = 0;
	virtual bool IsError( DWORD fdwErr = MOTION_ERR_ALL );
	virtual bool IsJoystickEnabled( );
	virtual bool IsInMotion( int nAxis )              = 0;
	virtual bool IsInMotion( void );
	static bool IsMotionEnabled();
	virtual bool IsTrackingError( void )              = 0;
	virtual bool IsOnFwLimit( int nAxis )             = 0;
	virtual bool IsOnBwLimit( int nAxis )             = 0;
	virtual bool IsAllSettled( )                      = 0;
	virtual bool IsSettled( int nAxis )               = 0;
	virtual bool IsSettled( void );

	virtual bool IsLastMotionCompleted()              = 0;
	virtual double GetRezeroSpeed( int nAxis )        = 0;


	//---- Actions ----
	virtual DWORD EnableJoystick( bool bEnable )      = 0;
	virtual DWORD Home( void );
	virtual DWORD Home( int nAxis );
	virtual DWORD Jog( double dfVel, int nAxis )      = 0;
	virtual DWORD Reset( void );
	virtual DWORD Rezero( void )                      = 0;
	virtual DWORD Rezero( int nAxis )                 = 0;
	virtual DWORD DeceleratedStop( int nAxis )        = 0;
	virtual DWORD DeceleratedStop( void )             = 0;
	virtual DWORD ImmediateStop( void )               = 0;

	virtual DWORD MoveRel( double dfPos, int nAxis );
	virtual DWORD MoveRelXY( double dfXPos, double dfYPos );
	virtual DWORD MoveTo( double dfPos, int nAxis );
	virtual DWORD MoveToXY( double dfXPos, double dfYPos );
	virtual void EnableJogging(bool bJogEnabled)     = 0;

	//---- Setting Options ----
	virtual DWORD ClearJSMovedFlag( void );
	virtual DWORD ConfigureHardware( void );
   void OnMotionDisabled();
	virtual DWORD SetJSMovedFlag( void );
	virtual DWORD SetHome( void );
	virtual DWORD SetHome( int nAxis );
	virtual DWORD SetHome( double dfPos, int nAxis );
	virtual DWORD SetLimits( double dfBWLimit, double dfFWLimit, int nAxis )	= 0;
	virtual DWORD SetSpeed( double dfSpeed, int nAxis )				= 0;
	virtual DWORD SetAccel( double dfAcDc, int nAxis )				= 0;

  	// Laser Probe Additions - [ RMW ]
	virtual double GetAnalogValue( int nChannelNumber ) = 0;
	virtual DWORD SetDIOBit(int nBitNumber) = 0;
   virtual DWORD ClearDIOBit(int nBitNumber)= 0;
   virtual DWORD InitializeCoordMotion(   ) = 0;
   virtual DWORD AddArcMove(double fRadius, double fStartAngle, double fDisplacementAngle, double fVS) = 0;
   virtual DWORD AddVectorMove(double fDX, double fDY, double fVS) = 0;
   virtual DWORD EndCoordinatedMotion()= 0;

   virtual bool IsCoordinatedMotionComplete()= 0;
   virtual DWORD ExecuteCoordinatedMotion()= 0;
	virtual DWORD TrackingOn()= 0;
   virtual DWORD TrackingOff()= 0;
	virtual DWORD AutoFocusLaser ( double fZRange, double fZSpeed ) = 0;

   virtual int IsTrackingOn() =0;
   virtual DWORD GetDIOBit(int nBitNumber) =0  ;
   virtual bool ActivateJoystick(bool bActivate, bool bHwJoystk){return true;}; //q-1768
   virtual bool ResetForRstop( ) {return false; }; // make base class fail.


};

//=============================================================================
//	Inline Methods
//=============================================================================
inline DWORD MOTION::clearErrorBits( DWORD dwErrCode )
{
	return( m_fdwError &= (~dwErrCode | MOTION_ERR_CRITICAL ) );
}

inline DWORD MOTION::setErrorBits( DWORD dwErrCode )
{
	return( m_fdwError |= dwErrCode );
}

inline bool MOTION::isAxisValid( int nIdx )
{
	if (nIdx >= 0 || nIdx < m_nMaxAxisCount)
	   return( TRUE );

	setErrorBits( MOTION_ERR_INVALID_PARAM );

	return( FALSE );
}

inline DWORD MOTION::ConfigureHardware( void )
{
	clearErrorBits( MOTION_ERR_ALL );
	return( setErrorBits( MOTION_ERR_FUNCTION_UNAVAIL ) );
}

inline DWORD MOTION::ClearJSMovedFlag( void )
{
	clearErrorBits( MOTION_ERR_ALL );
	return( setErrorBits( MOTION_ERR_FUNCTION_UNAVAIL ) );
}

inline DWORD MOTION::SetJSMovedFlag( void )
{
	clearErrorBits( MOTION_ERR_ALL );
	return( setErrorBits( MOTION_ERR_FUNCTION_UNAVAIL ) );
}

inline bool MOTION::GetJSMovedFlag( void )
{
	clearErrorBits( MOTION_ERR_ALL );
	setErrorBits( MOTION_ERR_FUNCTION_UNAVAIL );
	return( FALSE );
}

inline double MOTION::GetHome( int nAxis)
{
	if ( clearErrorBits( MOTION_ERR_ALL ) )
	   return( 0.0 );

	if (nAxis > m_nMaxAxisCount)
	{
	   setErrorBits( MOTION_ERR_INVALID_PARAM );
	   return( 0.0 );
	}
	else
	   return( m_adfHomePos[nAxis] );
}

inline bool MOTION::IsJoystickEnabled( void )
{
	return( m_bJoystickEnabled );
}

inline DWORD MOTION::GetError( void )
{
	return( m_fdwError );
}

inline bool MOTION::IsError( DWORD fdwErr )
{
	return( (m_fdwError & fdwErr) != 0L );
}

inline DWORD MOTION::MoveTo( double dfPos, int nAxis )
{
	if ( clearErrorBits(MOTION_ERR_ALL) )
	   return( m_fdwError );

	move( dfPos, nAxis, FALSE );
	return( m_fdwError );
}

inline DWORD MOTION::MoveToXY( double dfXPos, double dfYPos )
{
	if ( clearErrorBits(MOTION_ERR_ALL) )
	   return( m_fdwError );

	moveXY( dfXPos, dfYPos, FALSE );
	return( m_fdwError );
}

inline DWORD MOTION::MoveRel( double dfPos, int nAxis )
{
	if ( clearErrorBits(MOTION_ERR_ALL) )
	   return( m_fdwError );

	move( dfPos, nAxis, TRUE );
	return( m_fdwError );
}

inline DWORD MOTION::MoveRelXY( double dfXPos, double dfYPos )
{
	if ( clearErrorBits(MOTION_ERR_ALL) )
	   return( m_fdwError );

	moveXY( dfXPos, dfYPos, TRUE );
	return( m_fdwError );
}

#endif

//*** End MOTION.HPP ***