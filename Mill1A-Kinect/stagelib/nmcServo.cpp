//=============================================================================
//      
//      
//
//	File	:	NMCSERVO.cpp
//	Name	:	Don Uchiyama
//	Date	:	6/8/2001
//
//	Desc	: 	
//			
//			
//
//	Defines	:	
//
//=============================================================================

//=============================================================================
//	Includes
//=============================================================================
#include "stdafx.h"
#include <math.h>
#include <stdio.h>
//#include "path.h"
#include "sio_util.h"
#include "nmccom.h"
#include "picio.h"
#include "picservo.h"
//#include "ServoInf.h"
#include "nmcServo.h"

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
AxisInfo::AxisInfo()
{
	
	byAxis			= 0;
	byStopMode		= 0;
	byIOMode		= 0;
	byServoMode		= 0;
	byHomingMode	= 0;
	byStatusMode	= 0;

}
AxisInfo::~AxisInfo()
{
	byAxis			= 0;
	byStopMode		= 0;
	byIOMode		= 0;
	byServoMode		= 0;
	byHomingMode	= 0;
	byStatusMode	= 0;
	

}
NMCGainLimit::NMCGainLimit()
{
	siIntegralLimit	= 0; 
	byOutputLimit		= 255; 
	byCurrentLimit	= 0; 
	siErrorLimit		= 4000; 
	byServoRate		= 1;
	byDeadBand		= 0;
	
};
NMCGainLimit::~NMCGainLimit()
{

};
IOInfo::IOInfo()
{
	

}
IOInfo::~IOInfo()
{
	

}

NMCSERVO::NMCSERVO(   )
{
	m_iModules		  =0;
	m_iServoModules	  =0;
	m_iIOModules	  =0;

	for (int i = 0; i <= g_NumberOfAxis-1; i++)
	{
		m_lVelocity     [i]=10;
		m_lAcceleration [i]=10;
		m_NMCGainLimits [i].siIntegralLimit = 0;
		m_NMCGainLimits [i].byOutputLimit   = 255;
		m_NMCGainLimits [i].byCurrentLimit  = 0;
		m_NMCGainLimits [i].siErrorLimit    = 4000;
	    m_NMCGainLimits [i].byServoRate	  = 1;
	    m_NMCGainLimits [i].byDeadBand	  = 0;

	}

};

NMCSERVO::~NMCSERVO( void )
{
	
	if (m_iModules == 0) return;
//	NmcShutdown();
	
};
	
int  NMCSERVO::Initialize( const char * pComPort )
{
	char szMessage[80];
	AxisInfo Axis;
//Controllers on COM1, use 115200 baud
//Returns the number of modules found
	m_iModules = NmcInit((char *)pComPort, 19200);  
								    
//punt if no modules found
	if (m_iModules == 0) 
	{
		sprintf(szMessage,"NO MODULES FOUND");
		SimpleMsgBox(szMessage);
		return -1;
		
	}

//Display information on the modules found
	int i;
	for (i=0; i<m_iModules; i++)
	{
		if (NmcGetModType((byte)(i+1)) == SERVOMODTYPE)      
		{
			sprintf(szMessage,"Module %d: PIC-SERVO Controller", i+1);
			SimpleMsgBox(szMessage);
			Axis.byAxis = i+1;
			m_iServoModules++;
			m_Axis.push_back(Axis);
		}
  
		if (NmcGetModType((byte)(i+1)) == IOMODTYPE)
		{
			sprintf(szMessage,"Module %d: PIC-IO Controller", i+1);
			SimpleMsgBox(szMessage);
			m_IO[m_iIOModules++].byIO =i+1;
		}
	}
	
	if (m_Axis[0].byAxis == 0)
		return -1;

//Verify that we are talking to PIC-SERVO CMC modules
	for (i=0; i < m_iServoModules-1 ; i++)
	{
		if (NmcGetModVer(m_Axis[i].byAxis ) < 5)
		{
			SimpleMsgBox("Modules not PIC-SERVO CMC - Cannot continue demo.");
			return -1;
		}

	}

	SetDefaultIOMode();
	SetDefaultServoMode();
	SetDefaultHomeMode();
	SetDefaultStatusMode();
	SetDefaultStopMode();

	return m_iModules;

};
int	NMCSERVO::GetAxisInfo				(vector <AxisInfo> & AxisInfo)
{
	if (m_iServoModules == 0)
		return -1;

	AxisInfo = m_Axis;
	return 0;
	
};


//---- Querying ----

int NMCSERVO::GetMaxAccel			( eAxis nAxis, double & dAcceleration )
{

	return 0;
};
int NMCSERVO::GetMaxSpeed			( eAxis nAxis, double & dSpeed )
{
	
	return 0;
};
int NMCSERVO::GetMaxRezeroTimeout	( double & dTimeOut )
{
	
	
	return 0;
};
int NMCSERVO::GetMaxPos				( eAxis nAxis, double & dPosition )
{
	
	
	return 0;
};
int NMCSERVO::GetMinPos				( eAxis nAxis, double & dPosition )
{
	
	
	return 0;
};



BOOL   NMCSERVO::IsInMotion( eAxis nAxis )
{

	byte byStatus;
	NmcDefineStatus(m_Axis[nAxis].byAxis, SEND_AUX);
	NmcNoOp((byte) m_Axis[nAxis].byAxis);	//poll controller to get current status data
	byStatus = NmcGetStat((byte) m_Axis[nAxis].byAxis);
	if (!(byStatus & MOVE_DONE)) //wait for MOVE_DONE bit to go HIGH
	{
		Sleep (50);
		return TRUE;
	}
	else
	{
		Sleep (50);
		return FALSE;
	}
};
int   NMCSERVO::IsTrackingError( void )
{
	
	return 0;
};
// Original commands 

long NMCSERVO::GetCmdPosition( eAxis nAxis )
{
	return (ServoGetCmdPos((byte) m_Axis[nAxis].byAxis));
};
long NMCSERVO::GetCmdSpeed( eAxis nAxis ){
	
//	NmcDefineStatus((byte)m_Axis[nAxis].byAxis,SEND_VEL );
//	NmcReadStatus((byte)m_Axis[nAxis].byAxis,SEND_VEL );
	return (ServoGetCmdVel((byte)m_Axis[nAxis].byAxis));
};
unsigned int  NMCSERVO::GetCmdPwm ( eAxis nAxis )
{
	return (ServoGetCmdPwm((byte)m_Axis[nAxis].byAxis));
};
long NMCSERVO::GetCmdAccel( eAxis nAxis ){
	
	return (ServoGetCmdAcc((byte)m_Axis[nAxis].byAxis));
};


long NMCSERVO::GetSpeed( eAxis nAxis )
{
	NmcDefineStatus((byte)m_Axis[nAxis].byAxis,SEND_VEL );
	NmcReadStatus((byte)m_Axis[nAxis].byAxis, SEND_VEL );
	return (ServoGetVel((byte)m_Axis[nAxis].byAxis));
};

long NMCSERVO::GetAccel( eAxis nAxis )
{
	
	return (ServoGetCmdAcc((byte)m_Axis[nAxis].byAxis));
};
int  NMCSERVO::GetPositionError ( eAxis nAxis )
{
	NmcDefineStatus((byte)m_Axis[nAxis].byAxis, SEND_PERROR );
	NmcReadStatus((byte)m_Axis[nAxis].byAxis, SEND_PERROR );

	return (ServoGetPError((byte)m_Axis[nAxis].byAxis));
};
long  NMCSERVO::GetAD ( eAxis nAxis )
{
	NmcDefineStatus((byte)m_Axis[nAxis].byAxis, SEND_AD );
	NmcReadStatus((byte)m_Axis[nAxis].byAxis, SEND_AD );
	return (ServoGetAD((byte)m_Axis[nAxis].byAxis));
};
long NMCSERVO::GetHome( eAxis nAxis )
{
	NmcDefineStatus((byte)m_Axis[nAxis].byAxis,SEND_HOME );
	NmcReadStatus((byte)m_Axis[nAxis].byAxis,SEND_HOME );
	return (ServoGetHome((byte)m_Axis[nAxis].byAxis));
	
};
int  NMCSERVO::GetAux ( eAxis nAxis )
{
	NmcDefineStatus((byte)m_Axis[nAxis].byAxis, SEND_AUX );
	NmcReadStatus((byte)m_Axis[nAxis].byAxis, SEND_AUX );
	return (ServoGetAux((byte)m_Axis[nAxis].byAxis));
};
long  NMCSERVO::GetStopPosition ( eAxis nAxis )
{
	return (ServoGetStopPos((byte)m_Axis[nAxis].byAxis));
};
unsigned int  NMCSERVO::GetMoveCtrl ( eAxis nAxis )
{
	return (ServoGetMoveCtrl((byte)m_Axis[nAxis].byAxis));
};
unsigned int  NMCSERVO::GetStopCtrl ( eAxis nAxis )
{
	return (ServoGetStopCtrl((byte)m_Axis[nAxis].byAxis));
};
unsigned int  NMCSERVO::GetHomeCtrl ( eAxis nAxis )
{
	return (ServoGetHomeCtrl((byte)m_Axis[nAxis].byAxis));
};
unsigned int  NMCSERVO::GetIOCtrl ( eAxis nAxis )
{
	return (ServoGetIoCtrl((byte)m_Axis[nAxis].byAxis));
};

long NMCSERVO::GetPosition( eAxis nAxis )
{

	NmcDefineStatus((byte)m_Axis[nAxis].byAxis,SEND_POS );
	NmcReadStatus((byte)m_Axis[nAxis].byAxis,SEND_POS );
	return (ServoGetPos((byte)m_Axis[nAxis].byAxis));
};

int  NMCSERVO::GetRezeroStatus( void )
{
	
	return 0;
};

int  NMCSERVO::GetGain(  eAxis nAxis,
					     int & iProportional, 
						 int & iDifferential, 
						 int & iIntegral)
{
	
	short int siProp;
	short int siDiff;
	short int siInte;

	ServoGetGain(m_Axis[nAxis].byAxis,     //axis = 1
						&siProp, // Kp 
						&siDiff, // Kd
						&siInte,	    // Ki 
						&m_NMCGainLimits[nAxis].siIntegralLimit,        // IL
						&m_NMCGainLimits[nAxis].byOutputLimit,          // OL
						&m_NMCGainLimits[nAxis].byCurrentLimit,         // CL
						&m_NMCGainLimits[nAxis].siErrorLimit,		    // EL        
						&m_NMCGainLimits[nAxis].byServoRate,		//SR = 1
						&m_NMCGainLimits[nAxis].byDeadBand);		//DC = 0
          
	iProportional =(int)siProp;
	iIntegral = (int)siInte;
	iDifferential = (int)siDiff;
	return 0;
};
int  NMCSERVO::SetGain(  eAxis nAxis,
					     int iProportional, 
						 int iDifferential, 
						 int iIntegral)
{
	
	m_bError=ServoSetGain2(m_Axis[nAxis].byAxis,     //axis = 1
						(short int)iProportional, // Kp 
						(short int)iDifferential, // Kd
						(short int)iIntegral,	    // Ki 
						m_NMCGainLimits[nAxis].siIntegralLimit,        // IL
						m_NMCGainLimits[nAxis].byOutputLimit,          // OL
						m_NMCGainLimits[nAxis].byCurrentLimit,         // CL
						m_NMCGainLimits[nAxis].siErrorLimit,		    // EL        
						1,		//SR = 1
						0,1);		//DC = 0
             
	return (int)m_bError;
};

int  NMCSERVO::SetGainLimits( eAxis nAxis, 
							  int iIntegralLimit, 
						      int iOutputLimit, 
							  int iCurrentLimit, 
							  int iErrorLimit,
							  int iServoRate, 
							  int iDeadBand)
{
	
	m_NMCGainLimits[nAxis].siIntegralLimit = (short int)iIntegralLimit;
	m_NMCGainLimits[nAxis].byOutputLimit   = (byte)iOutputLimit;
	m_NMCGainLimits[nAxis].byCurrentLimit  = (byte)iCurrentLimit;
	m_NMCGainLimits[nAxis].siErrorLimit    = (short int)iErrorLimit;
	m_NMCGainLimits[nAxis].byServoRate     = (byte)iServoRate;
	m_NMCGainLimits[nAxis].byDeadBand      = (byte)iDeadBand;

	return 0;
};
int  NMCSERVO::GetGainLimits( eAxis nAxis,
							  int & iIntegralLimit, 
						      int & iOutputLimit, 
							  int & iCurrentLimit, 
							  int & iErrorLimit,
							  int & iServoRate, 
							  int & iDeadBand)
{
	
	iIntegralLimit    = m_NMCGainLimits[nAxis].siIntegralLimit;
	iOutputLimit      = m_NMCGainLimits[nAxis].byOutputLimit;
	iCurrentLimit     = m_NMCGainLimits[nAxis].byCurrentLimit;
	iErrorLimit       = m_NMCGainLimits[nAxis].siErrorLimit;
	iServoRate        = m_NMCGainLimits[nAxis].byServoRate;
	iDeadBand		  = m_NMCGainLimits[nAxis].byDeadBand;

	return 0;
};

//---- Actions ----

int  NMCSERVO::ResetPosition( eAxis nAxis )
{
	m_bError=ServoResetPos((byte)m_Axis[nAxis].byAxis);
	return m_bError;
}
int  NMCSERVO::ResetRelHome( eAxis nAxis )
{
	return ServoResetRelHome(m_Axis[nAxis].byAxis);
};
int  NMCSERVO::SetPosition ( eAxis nAxis, long lPosition )
{
	return ServoSetPos(m_Axis[nAxis].byAxis, lPosition);	
};
int  NMCSERVO::ClearBits( eAxis nAxis )
{
	m_bError=ServoClearBits(m_Axis[nAxis].byAxis);
	return m_bError;	
};
int  NMCSERVO::StopMotor (eAxis nAxis, int nMode)
{
	m_bError=ServoStopMotor(m_Axis[nAxis].byAxis, m_Axis[nAxis].byStopMode |nMode );
	return (int)m_bError;
};
int  NMCSERVO::StopHere (eAxis nAxis, int nMode, long lPosition)
{
	m_bError=ServoStopHere(m_Axis[nAxis].byAxis, (byte)nMode ,lPosition);
	return 0;
};

int NMCSERVO::StartMove(int iGroupAddr, int iGroupLeader)
{
	return ServoStartMove((byte) iGroupAddr, (byte) iGroupLeader);	
};	
int NMCSERVO::SetHoming(eAxis nAxis, int iMode)
{
	return ServoSetHoming(m_Axis[nAxis].byAxis, (byte)iMode);	
};	
int NMCSERVO::HardReset(eAxis nAxis, int iMode)
{
	return ServoHardReset((byte)m_Axis[nAxis].byAxis, (byte) iMode);	
};	
int  NMCSERVO::Rezero( eAxis nAxis )
{

	byte byStatus;
	long lPosition;
	long lHomePosition;

// Get Status for Home in Progress
	NmcDefineStatus(m_Axis[nAxis].byAxis, SEND_AUX);

// Find the position at limit 1
	ServoClearBits(m_Axis[nAxis].byAxis);
	ServoResetPos(m_Axis[nAxis].byAxis );
	ServoSetHoming(m_Axis[nAxis].byAxis, ON_LIMIT1 | ON_LIMIT2 | HOME_STOP_ABRUPT );
	lPosition = 1600000;	

	ServoResetPos(m_Axis[nAxis].byAxis );
	MoveTo( lPosition, nAxis );	

	do {
		NmcNoOp((byte)m_Axis[nAxis].byAxis);	
		byStatus = NmcGetStat((byte)m_Axis[nAxis].byAxis);
	}
    while (( HOME_IN_PROG & byStatus ) && (LIMIT1 & byStatus ));

	while (IsInMotion((eAxis)nAxis));
	Sleep(2000);

	ServoClearBits(m_Axis[nAxis].byAxis);
	ServoSetHoming(m_Axis[nAxis].byAxis,  ON_LIMIT2 | HOME_STOP_ABRUPT );
	ServoResetPos(m_Axis[nAxis].byAxis );

	MoveTo( -lPosition, nAxis );	
	do {
		NmcNoOp((byte) nAxis);	
		byStatus = NmcGetStat((byte) nAxis);
	}
	while (( HOME_IN_PROG & byStatus ) && (LIMIT2 & byStatus ));
	Sleep(2000);
		
	lHomePosition=GetHome(  nAxis );
	lPosition=GetPosition(  nAxis );	

	MoveTo(lPosition/2, nAxis);
	while (IsInMotion( nAxis ));
	return 0;
};
int  NMCSERVO::StopMotor (eAxis nAxis)
{
	m_bError=ServoStopMotor(m_Axis[nAxis].byAxis, m_Axis[nAxis].byStopMode );
	return (int)m_bError;
};

//---- Setting Options ----

int  NMCSERVO::SetSpeed(eAxis nAxis, long lSpeed )
{
	m_lVelocity [nAxis] = lSpeed;
	return 0;
};
int  NMCSERVO::SetAccel(eAxis nAxis, long lAcceleration )
{
	
	m_lAcceleration [nAxis] = lAcceleration;
	return 0;
};
int  NMCSERVO::SetDownloadFileName( TCHAR *pszFileName )
{
	
	return 0;
};

int   NMCSERVO::MoveTo( long lPos, eAxis nAxis )
{
	
	m_bError=ServoLoadTraj(m_Axis[nAxis].byAxis, 
		               m_Axis[nAxis].byServoMode | START_NOW , 
					   lPos, 
					   m_lVelocity[nAxis], 
					   m_lAcceleration[nAxis], 
					   0);	

	return (int)!m_bError;
};	
int   NMCSERVO::MoveRel( long lPos, eAxis nAxis)
{
		
	m_bError=ServoLoadTraj(m_Axis[nAxis].byAxis, 
		           m_Axis[nAxis].byServoMode | START_NOW | MOVE_REL , 
				   lPos, 
				   m_lVelocity[nAxis], 
				   m_lAcceleration[nAxis], 
				   0);	
	
	return (int)m_bError;
};
int   NMCSERVO::EnableAmp    (eAxis nAxis)
{
	SetStopMode(nAxis, AMP_ENABLE | MOTOR_OFF);
	SetStopMode(nAxis, AMP_ENABLE | STOP_ABRUPT);
	StopMotor(nAxis);
	return 0;
};
int   NMCSERVO::DisableAmp   (eAxis nAxis)
{
	
	m_Axis[nAxis].byStopMode  = m_Axis[nAxis].byStopMode | 
		                                        !AMP_ENABLE | 
												ADV_FEATURE;
	return 0;
};
	
int   NMCSERVO::SetDefaultIOMode()
{
	int iAddress;
	for (iAddress=0; iAddress < m_iModules; iAddress++)
		m_Axis[iAddress].byIOMode     = IO1_IN | IO2_IN;

	return 0;	
};
int   NMCSERVO::SetDefaultServoMode()
{
	int iAddress;
	for (iAddress=0; iAddress < m_iModules; iAddress++)
		m_Axis[iAddress].byServoMode = LOAD_POS | LOAD_VEL | LOAD_ACC | ENABLE_SERVO;
	return 0;	
};
int   NMCSERVO::SetDefaultHomeMode()
{

	int iAddress;
	for (iAddress=0; iAddress < m_iModules-1; iAddress++)
		m_Axis[iAddress].byHomingMode = ON_LIMIT1 | ON_LIMIT2 | ON_POS_ERR | ON_CUR_ERR;

	return 0;	
};
int   NMCSERVO::SetDefaultStatusMode()
{
	
	int iAddress;
	for (iAddress=0; iAddress < m_iModules-1; iAddress++)
		m_Axis[iAddress].byStatusMode= SEND_POS | SEND_VEL | SEND_ID;

	return 0;		
};
int   NMCSERVO::SetDefaultStopMode()
{
	int iAddress;
	for (iAddress=0; iAddress < m_iModules-1; iAddress++)
		m_Axis[iAddress].byStopMode  = STOP_SMOOTH | AMP_ENABLE | ADV_FEATURE;

	return 0;		
};

int   NMCSERVO::SetStopMode(eAxis nAxis, byte byMode)
{
	m_Axis[nAxis].byStopMode   = m_Axis[nAxis].byStopMode | byMode;	
	return 0;	

};
int   NMCSERVO::SetServoMode(eAxis nAxis, byte byMode)
{
	m_Axis[nAxis].byServoMode  = m_Axis[nAxis].byServoMode | byMode;
	return 0;	
};
int   NMCSERVO::SetIOMode(eAxis nAxis, byte byMode)
{
	
	m_Axis[nAxis].byIOMode     = m_Axis[nAxis].byIOMode | byMode;
	return 0;	
};

int   NMCSERVO::SetIOCtrl			( eAxis nAxis, int iMode )
{
	m_bError=ServoSetIoCtrl(byte(nAxis), byte(iMode));	
	return m_bError;

};

int   NMCSERVO::SetHomeMode(eAxis nAxis, byte byMode)
{
	
	m_Axis[nAxis].byHomingMode = byMode;
	return 0;		
};

int   NMCSERVO::SetStatusMode(eAxis nAxis, byte byMode)
{

	m_Axis[nAxis].byStatusMode = byMode;
	return 0;		
};



//*** End NMCSERVO.HPP ***