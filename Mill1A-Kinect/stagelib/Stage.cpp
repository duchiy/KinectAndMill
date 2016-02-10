#include "stdafx.h"
#include "Stage.h"
#include "nmcServo.h"
GainLimit::GainLimit()
{
	dIntegralLimit		= 0.0; 
	dOutputLimit		= 255.0; 
	dCurrentLimit		= 0.0; 
	dErrorLimit			= 4000.0; 
	dServoRate			= 1.0;
	dDeadBand			= 0.0;
};
GainLimit::~GainLimit()
{
	
	dIntegralLimit		= 0.0; 
	dOutputLimit		= 255.0; 
	dCurrentLimit		= 0.0; 
	dErrorLimit			= 4000.0; 
	dServoRate			= 1.0;
	dDeadBand			= 0.0;
	
};

Stage::Stage(  )
{
	for (int i=0; i<=g_NumberOfAxis; i++)
	{
		m_dVelocity				[i]=10;
		m_dAcceleration			[i]=20;
		m_dEncoderCountsQuad	[i]=10;
		m_dLeadScrewTreadsInch	[i]=1.0;
		m_dMotorGearRatio		[i]=1.0;
		m_dPulleyRatio			[i]=1.0;

	}
	m_rotateAngle =0.0;
};
Stage::Stage( const char *pComPort  )
{
	Stage::Stage();
	Stage::Initialize( pComPort  );
};

int	Stage::Initialize( const char *pComPort  )
{
	
	m_iServoModules=m_NMCServo.Initialize(pComPort);
	if (m_iServoModules < 3)
		return -1;

	m_NMCServo.GetAxisInfo(m_Axis);

	vector<int> iAxis;
	m_NMCServo.GetAxisInfo(m_Axis);
	GetAxis(iAxis);
//	m_Path.SetAxis(iAxis);

	return m_iServoModules;	
};
int	Stage::Initialize( const char *pComPort, Path & path )
{
	
	m_iServoModules=m_NMCServo.Initialize(pComPort);
	if (m_iServoModules < 3)
		return -1;

	m_NMCServo.GetAxisInfo(m_Axis);

	vector<int> iAxis;
	m_NMCServo.GetAxisInfo(m_Axis);
	GetAxis(iAxis);
	path.SetAxis(iAxis);

	return m_iServoModules;	
};
Stage & Stage::operator=(Stage & stage)
{
		return stage;	

}

Stage::~Stage( void )
{
	
};	

int Stage::Rotate(double & x, double & y, double & z)
{
	Vector4 vt, vt1;
	vt.set(x,y,z, 1);
	m_mt.multiply(vt1, vt);
	x=vt1.getElement(0);y=vt1.getElement(1);z=vt1.getElement(2);
	return 0;
};
int Stage::Rotate(vector<PathPoint> & Pathpts)
{
	int numberOfPts = Pathpts.size();
	vector<double> initPt(3);
	vector<double> transPt(3);

	for (int i=0; i <= Pathpts.size()-1; i++)
	{
//		transPt.clear();
//		initPt.clear();
		initPt[0] = Pathpts[i].X; initPt[1] = Pathpts[i].Y; initPt[2] = Pathpts[i].Z;
		transPt=m_mt.transformPoint(initPt);
		Pathpts[i].X = transPt[0]; Pathpts[i].Y = transPt[1]; Pathpts[i].Y = transPt[1];
	}
	return 0;
};

int Stage::GetAxis	(vector<int> & iAxis)
{
	int i=0;
	for (i=0 ;i <= m_Axis.size()-1 ;i++)
	{
		iAxis.push_back(m_Axis[i].byAxis);
	}
	return 0;
};

int Stage::GetGain		  ( eAxis nAxis,
							int & siProportional, 
							int & siDifferential, 
							int & siIntegral)
{
	m_iError=m_NMCServo.GetGain(nAxis, siProportional, siDifferential, siIntegral);
	return 0;
};

int Stage::SetGain		    ( eAxis nAxis,
		                      int siProportional, 
						  	  int siDifferential, 
							  int siIntegral )
{
	
	m_iError=m_NMCServo.SetGain(nAxis, siProportional, siDifferential, siIntegral);
	return 0;	
};

int Stage::GetGainLimits	( eAxis nAxis,
		                      GainLimit & GainLimits)
{
	m_iError=m_NMCServo.GetGainLimits(	nAxis, 
		                                (int &)m_GainLimits[nAxis].dIntegralLimit,
										(int &)m_GainLimits[nAxis].dOutputLimit, 
										(int &)m_GainLimits[nAxis].dCurrentLimit, 
										(int &)m_GainLimits[nAxis].dErrorLimit, 
										(int &)m_GainLimits[nAxis].dServoRate,
										(int &)m_GainLimits[nAxis].dDeadBand);
	 GainLimits=m_GainLimits[nAxis];
	
	return 0;	
};

int Stage::SetGainLimits	( eAxis nAxis,
		                      GainLimit GainLimits)
{
	
	m_GainLimits[nAxis] = GainLimits;

	m_iError=m_NMCServo.SetGainLimits(	nAxis,
		                                m_GainLimits[nAxis].dIntegralLimit, 
										m_GainLimits[nAxis].dOutputLimit, 
										m_GainLimits[nAxis].dCurrentLimit, 
										m_GainLimits[nAxis].dErrorLimit, 
										m_GainLimits[nAxis].dServoRate,
										m_GainLimits[nAxis].dDeadBand);
	return 0;		
};

BOOL Stage::IsInMotion( void )
{

    BOOL bInMotion;
    int iAddress;
	bInMotion = FALSE;

	for (iAddress = 0;iAddress <= m_iServoModules-1; iAddress++)
	{
		bInMotion= bInMotion | m_NMCServo.IsInMotion((eAxis)iAddress);
	}

	return bInMotion;
};
void  Stage::Rotate(double angle)
{
	m_rotateAngle = angle*PI/180.0;
	m_mt.rotateZ(m_rotateAngle);
};

int Stage::GetPos				( double & XPos,	double & YPos,		double & ZPos)
{
	XPos=m_NMCServo.GetPosition( eXAxis );
	YPos=m_NMCServo.GetPosition( eYAxis );
	ZPos=m_NMCServo.GetPosition( eZAxis );

	XPos=XPos/m_fScale[eXAxis];
	YPos=YPos/m_fScale[eYAxis];
	ZPos=ZPos/m_fScale[eZAxis];
	
	if (m_rotateAngle != 0.0)
	{
		Vector4 vt, vt1;
		vt.set(XPos,YPos,ZPos, 1);
		m_mt.multiply(vt1, vt);
		XPos=vt1.getElement(0);YPos=vt1.getElement(1);ZPos=vt1.getElement(2);
	}

	return 0;
};
int Stage::GetCmdPos			( double & XPos,	double & YPos,		double & ZPos )
{
	XPos=m_NMCServo.GetCmdPosition( eXAxis );
	YPos=m_NMCServo.GetCmdPosition( eYAxis );
	ZPos=m_NMCServo.GetCmdPosition( eZAxis );

	XPos=XPos/m_fScale[eXAxis];
	YPos=YPos/m_fScale[eYAxis];
	ZPos=ZPos/m_fScale[eZAxis];
	
	return 0;
};
int Stage::GetPosError			( double & XError,	double & YError,	double & ZError )
{
	XError=(double)m_NMCServo.GetPositionError(eXAxis);
	YError=(double)m_NMCServo.GetPositionError(eYAxis);
	ZError=(double)m_NMCServo.GetPositionError(eZAxis);
	return 0;
};
int Stage::GetCmdVel			( double & XVel,	double & YVel,		double & ZVel )
{
	XVel=m_NMCServo.GetCmdSpeed(eXAxis);
	YVel=m_NMCServo.GetCmdSpeed(eYAxis);
	ZVel=m_NMCServo.GetCmdSpeed(eZAxis);
	XVel=XVel/m_fScale[eXAxis];
	YVel=YVel/m_fScale[eYAxis];
	ZVel=ZVel/m_fScale[eZAxis];
	return 0;
};
int Stage::GetSpeed			( double & XVel,	double & YVel,		double & ZVel )
{

	XVel=m_NMCServo.GetSpeed(eXAxis);
	YVel=m_NMCServo.GetSpeed(eYAxis);
	ZVel=m_NMCServo.GetSpeed(eZAxis);
	XVel=XVel/m_fScale[eXAxis];
	YVel=YVel/m_fScale[eYAxis];
	ZVel=ZVel/m_fScale[eZAxis];
	return 0;
};

int Stage::GetCmdAccel			( double & XAccel,	double & YAccel,	double & ZAccel )
{
	XAccel=m_NMCServo.GetCmdAccel(eXAxis);
	YAccel=m_NMCServo.GetCmdAccel(eYAxis);
	ZAccel=m_NMCServo.GetCmdAccel(eZAxis);
	XAccel=XAccel/m_fScale[eXAxis];
	YAccel=YAccel/m_fScale[eYAxis];
	ZAccel=ZAccel/m_fScale[eZAxis];
	return 0;
};
int Stage::GetCmdPwm			( double & XPwm,	double & YPwm,		double & ZPwm )
{
	
	XPwm=m_NMCServo.GetCmdPwm(eXAxis);
	YPwm=m_NMCServo.GetCmdPwm(eYAxis);
	ZPwm=m_NMCServo.GetCmdPwm(eZAxis);
	return 0;
};
int Stage::GetAD				( double & XAD,		double & YAD,		double & ZAD )
{
	
	XAD=m_NMCServo.GetAD(eXAxis);
	YAD=m_NMCServo.GetAD(eYAxis);
	ZAD=m_NMCServo.GetAD(eZAxis);
	return 0;
};
int Stage::GetHome				( double & XPos,	double & YPos,		double & ZPos )
{
	XPos=m_NMCServo.GetHome( eXAxis );
	YPos=m_NMCServo.GetHome( eYAxis );
	ZPos=m_NMCServo.GetHome( eZAxis );
	
	XPos=XPos/m_fScale[eXAxis];
	YPos=YPos/m_fScale[eYAxis];
	ZPos=ZPos/m_fScale[eZAxis];

	return 0;
};
int Stage::GetStopPos			( double & XPos,	double & YPos,		double & ZPos )
{
	XPos=m_NMCServo.GetStopPosition( eXAxis );
	YPos=m_NMCServo.GetStopPosition( eYAxis );
	ZPos=m_NMCServo.GetStopPosition( eZAxis );

	XPos=XPos/m_fScale[eXAxis];
	YPos=YPos/m_fScale[eYAxis];
	ZPos=ZPos/m_fScale[eZAxis];

	return 0;
};

int Stage::StopHere			( double lXPos, double lYPos, double lZPos  )
{
	m_iError=0;
	m_iError=m_NMCServo.StopHere(eXAxis, m_Axis[0].byStopMode,lXPos);
	m_iError=m_NMCServo.StopHere(eYAxis, m_Axis[1].byStopMode,lYPos);
	m_iError=m_NMCServo.StopHere(eZAxis, m_Axis[2].byStopMode,lZPos);

	return m_iError;
};
int Stage::StopMotor			(  void  )
{
	
	int iAddress;

	for (iAddress=0;iAddress<=m_iServoModules-1;iAddress++)
		m_iError=m_NMCServo.StopMotor((eAxis)iAddress, 
		                  (byte)m_Axis[iAddress].byStopMode);
	
	return m_iError;
	
	
};
int Stage::StopMotor			( eAxis nAxis, int nMode )
{
	m_iError=m_NMCServo.StopMotor((eAxis)nAxis, (byte)nMode);
	return 0;
};

int Stage::EnableAmp(void)
{
	
	int iAddress;
	for (iAddress=0; iAddress <= m_iServoModules-1; iAddress++)
	{
		m_NMCServo.EnableAmp((eAxis)iAddress);
	}
	StopMotor();
	return 0;		
};
int  Stage::DisableAmp()
{
	int iAddress;
	for (iAddress=0; iAddress <= m_iServoModules-1; iAddress++)
		m_NMCServo.DisableAmp((eAxis)iAddress);
	return 0;
};
int Stage::Rezero				( void )
{
	
	return 0;
};
int Stage::ResetPos			( void )
{
	int iAddress;
	for (iAddress=0; iAddress <= m_iServoModules-1; iAddress++)
	{
		m_iError=m_NMCServo.ClearBits	  ( (eAxis)iAddress );
		m_iError=m_NMCServo.ResetPosition( (eAxis)iAddress );
	}
	return 0;
};
int Stage::ResetRelHome		( void )
{
	int iAddress;
	for (iAddress=0; iAddress <= m_iServoModules-1; iAddress++)
		m_iError=m_NMCServo.ResetRelHome( (eAxis)iAddress );

	return 0;
};
int Stage::MoveTo				( double lPos, int Axis, bool WithWait )
{
	double lXPos, lYPos, lZPos;

	GetPos(lXPos, lYPos, lZPos);
	switch (Axis)
	{
		// X-Axis
		case 0:
			if (m_rotateAngle != 0.0)
			{
				Rotate(lPos, lYPos, lZPos);
			}
			MoveTo(lPos, lYPos, lZPos);
			break;	
		// Y-Axis
		case 1:
			if (m_rotateAngle != 0.0)
			{
				Rotate(lXPos, lPos, lZPos);
			}
			MoveTo(lXPos, lPos, lZPos);
			break;	
		// Z-Axis
		case 2:
			MoveTo(lXPos, lYPos, lPos);
			break;
		
		default:
			return -1;
	}
	bool bInMotion = false;
	if (WithWait)
	{
		do{
			bInMotion=IsInMotion();
		}while(bInMotion);
	}
	
	return 0;	
};
int Stage::MoveTo				( double lXPos, double lYPos, bool WithWait )
{
	double x,y,z;
	if (m_rotateAngle != 0.0)
	{
		Rotate(lXPos, lYPos, z=0);
	}

	GetPos(x, y, z);
	MoveTo(lXPos, lYPos, z);
	bool bInMotion = false;
	if (WithWait)
	{
		do{
			bInMotion=IsInMotion();
		}while(bInMotion);
	}
	return 0;	
};

int Stage::MoveTo				( double lXPos, double lYPos, double lZPos, bool WithWait )
{
	m_IO.WriteCmd(enStageMove);
	m_IO.WriteXYZ(lXPos, lYPos, lZPos);
	
	if (m_rotateAngle != 0.0)
	{
		Rotate(lXPos, lYPos, lZPos);
	}

	MoveTo(lXPos, lYPos, lZPos);
	bool bInMotion = false;
	if (WithWait)
	{
		do{
			bInMotion=IsInMotion();
		}while(bInMotion);
	}
	return 0;
};

int Stage::MoveTo				( double XPos, double YPos, double ZPos )
{

	XPos=XPos*m_fScale[eXAxis];
	YPos=YPos*m_fScale[eYAxis];
	ZPos=ZPos*m_fScale[eZAxis];
	m_iError=m_NMCServo.MoveTo( XPos, eXAxis );
	m_iError=m_NMCServo.MoveTo( YPos, eYAxis );
	m_iError=m_NMCServo.MoveTo( ZPos, eZAxis );

	return 0;
};
int Stage::MoveRel				( double XPos, double YPos, double ZPos, bool WithWait )
{
	bool bInMotion = false;

	MoveRel(XPos, YPos, ZPos);
	if (WithWait)
	{
		do{
			bInMotion=IsInMotion();
		}while(bInMotion);
	}
	return 0;
};
int Stage::MoveRel				( double XPos, double YPos, double ZPos )
{
	XPos=XPos*m_fScale[eXAxis];
	YPos=YPos*m_fScale[eYAxis];
	ZPos=ZPos*m_fScale[eZAxis];

	m_iError=m_NMCServo.MoveRel( XPos, eXAxis );
	m_iError=m_NMCServo.MoveRel( YPos, eYAxis );
	m_iError=m_NMCServo.MoveRel( ZPos, eZAxis );

	return 0;
};
void Stage::EnableJogging		( bool bJogEnabled)
{
	
};
int Stage::SetEncoderCounts    ( eAxis nAxis,	double CountsPerQuadature )
{
	m_dEncoderCountsQuad[nAxis] = 4.0*CountsPerQuadature;
	SetScale();
	return 0;
};
int Stage::SetLeadScrewPitch   ( eAxis nAxis,  double ThreadPerInch )
{
	m_dLeadScrewTreadsInch[nAxis] = ThreadPerInch;
	SetScale();
	return 0;
};
int Stage::SetMotorGearRatio	( eAxis nAxis,	double GearRatio)
{
	m_dMotorGearRatio[nAxis] = GearRatio;
	SetScale();
	return 0;
};	
int Stage::SetPulleyRatio	( eAxis nAxis,	double PulleyRatio)
{
	m_dPulleyRatio	[nAxis] = PulleyRatio;
	SetScale();
	return 0;
};	
int Stage::GetEncoderCounts    ( eAxis nAxis,	double & CountsPerQuadature )
{
	CountsPerQuadature=m_dEncoderCountsQuad[nAxis];
	return 0;
};
int Stage::GetLeadScrewPitch   ( eAxis nAxis,  double & ThreadPerInch )
{
	ThreadPerInch= m_dLeadScrewTreadsInch[nAxis];
	return 0;
};
int Stage::GetMotorGearRatio	( eAxis nAxis,	double & GearRatio)
{
	GearRatio= m_dMotorGearRatio[nAxis];
	return 0;
};	
int Stage::GetPulleyRatio	( eAxis nAxis,	double & PulleyRatio)
{
	PulleyRatio=m_dMotorGearRatio	[nAxis];
	return 0;
};	

int Stage::SetPos				( double XPos, double YPos, double ZPos )
{
	XPos=XPos*m_fScale[eXAxis];
	YPos=YPos*m_fScale[eYAxis];
	ZPos=ZPos*m_fScale[eZAxis];
	
	m_iError=m_NMCServo.SetPosition(eXAxis, XPos);
	m_iError=m_NMCServo.SetPosition(eYAxis, YPos);
	m_iError=m_NMCServo.SetPosition(eZAxis, ZPos);

	return 0;
};
int Stage::SetPos				( eAxis nAxis,  double lPos )
{
	m_iError=m_NMCServo.SetPosition(nAxis, lPos);
	return 	m_iError;
};

int Stage::SetVel				( double lXVel, double lYVel, double lZVel )
{
	SetVel(eXAxis, lXVel);
	SetVel(eYAxis, lYVel);
	SetVel(eZAxis, lZVel);
	return 0;
};
int Stage::SetVel				( eAxis nAxis,  double lVel )
{
	m_dVelocity [nAxis] = lVel*m_fScale[nAxis];
	m_NMCServo.SetSpeed(nAxis, (long )m_dVelocity [nAxis]);
	return 0;
};
int Stage::SetAccel			( double lXAcc, double lYAcc, double lZAcc)
{
	SetAccel(eXAxis, lXAcc);
	SetAccel(eYAxis, lYAcc);
	SetAccel(eZAxis, lZAcc);
	return 0;
};
int Stage::SetAccel			( eAxis nAxis,  double lAcc)
{
	m_dAcceleration [nAxis] = lAcc*m_fScale[nAxis];
	m_NMCServo.SetAccel(nAxis, (long )m_dAcceleration [nAxis]);
	return 0;
};

int Stage::SetDownloadFileName	( TCHAR *pszFileName )
{
	
	return 0;
};
int Stage::EndCoordMotion()
{
	
	return 0;
};
int Stage::IsCoordMotionComplete()
{
	do
	{
		NmcNoOp((byte)m_Axis[0].byAxis);   //retrieve current status data
	}
	while ( ServoGetAux((byte)m_Axis[0].byAxis) & PATH_MODE );   //poll while still in path mode	
	
	return 0;
};
int Stage::ExecuteCoordMotion()
{
	int iNumPoints;
	vector<PathPoint> vPoint;
	vector<double> dPathLength;
    SetPathStatus();
	InitPathLength(	dPathLength );
	InitCoordMotion();

//	m_Path.WritePathFile(m_IO.GetOutFile());
	m_IO.WriteCmd(enExecutePath);
//	InitializeCoordMotion(1,2,3);
//
//Download path points to the PIC-SERVO CMC modules until all points are
//  downloaded.  Motion will begin automatically when the minimum number
//  of path points have been loaded.
//
	iNumPoints=CreatePathPoints(vPoint, dPathLength);
//	iNumPoints=g_vPath.SavePathToFile("C:\Program Files\Mill\dude.txt", vPoint);
	iNumPoints=AddPathPoints(vPoint) ;
	IsCoordMotionComplete();
//	m_Path.SavePathToFile(vPoint);
	vPoint.clear();
	return 0;
};
int Stage::ExecuteCoordMotion(Path & vPath)
{
	int iNumPoints;
	vector<PathPoint> vPoint;
	vector<double> dPathLength;
    SetPathStatus();
	vPath.InitPathLength(	dPathLength );
	InitCoordMotion();
	double pathSpeed;
	int    pathFreq;
	int maxPointsToStore;

	vPath.GetCmdPathSpeed(pathSpeed);
	vPath.GetPathFreq(pathFreq);
	vPath.GetMaxPointsToStore(maxPointsToStore);

//	InitializeCoordMotion(1,2,3);
//
//Download path points to the PIC-SERVO CMC modules until all points are
//  downloaded.  Motion will begin automatically when the minimum number
//  of path points have been loaded.
//
	iNumPoints=vPath.CreatePathPointsP(vPoint, dPathLength);
//	iNumPoints=g_vPath.SavePathToFile("C:\Program Files\Mill\dude.txt", vPoint);
	if (m_rotateAngle != 0.0)
	{
		Rotate(vPoint);
	}

	iNumPoints=AddPathPoints(vPoint,  maxPointsToStore, pathSpeed, pathFreq);
	IsCoordMotionComplete();
//	m_Path.SavePathToFile(vPoint);
	vPoint.clear();
	return 0;
};
int Stage::ExecuteCoordMotionOutputPtsOnly()
{
	int iNumPoints;
	vector<PathPoint> vPoint;
	vector<double> dPathLength;
	InitPathLength(	dPathLength );

	iNumPoints=CreatePathPoints(vPoint, dPathLength);
//	m_Path.SavePathToFile(vPoint);
	vPoint.clear();
	return 0;
};
int Stage::ClearCoordMotion()
{
	if (m_iServoModules == 0) 
		return -1;
	NmcShutdown();
	return 0;
};

int Stage::StopResetMotors()
{
	vector<AxisInfo>::iterator AxisItr;

	if ( CheckAxis() != 0 )
		return eInvalidAxis;

	for (AxisItr = m_Axis.begin() ; AxisItr != m_Axis.end(); AxisItr++)
	{
		ServoStopMotor(AxisItr->byAxis, AMP_ENABLE | STOP_ABRUPT | ADV_FEATURE);
	}
//	ServoStopMotor(m_Axis[0].byAxis, AMP_ENABLE | STOP_ABRUPT | ADV_FEATURE);
//	ServoStopMotor(m_Axis[1].byAxis, AMP_ENABLE | STOP_ABRUPT | ADV_FEATURE);
//	ServoStopMotor(m_Axis[2].byAxis, AMP_ENABLE | STOP_ABRUPT | ADV_FEATURE);

//Reset position counters to zero:
	ServoResetPos(m_Axis[0].byAxis);
	ServoResetPos(m_Axis[1].byAxis);
	ServoResetPos(m_Axis[2].byAxis);
	return 0;
};
int Stage::SetGroupAddress(int group, eAxis leader)
{
	bool		xLeader;
	bool		yLeader;
	bool		zLeader;

	m_group=(byte)group;
	xLeader= false;
	yLeader= false;
	zLeader= false;
	if ( leader == eXAxis )
	{
		m_leader = 	m_Axis[eXAxis].byAxis;
		xLeader=true;
	}

	if ( leader == eYAxis )
	{
		m_leader = 	m_Axis[eYAxis].byAxis;
		yLeader=true;
	}

	if ( leader == eZAxis )
	{
		m_leader = 	m_Axis[eZAxis].byAxis;
		zLeader=true;
	}
	
	if ( CheckAxis() != 0 )
		return eInvalidAxis;

	vector <AxisInfo>::iterator AxisItr;

	for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
	{
		if (m_Axis[leader].byAxis == AxisItr->byAxis)	
			NmcSetGroupAddr(AxisItr->byAxis, m_group, true);    //x axis is the group leader
		else
			NmcSetGroupAddr(AxisItr->byAxis, m_group, false);
	};

	return 0;	
};

int Stage::SetPathStatus()
{
	
	if ( CheckAxis() != 0 )
		return eInvalidAxis;

	vector <AxisInfo>::iterator AxisItr;
	for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
	{
		NmcDefineStatus(AxisItr->byAxis, SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX);
	}
	return 0;
};

int Stage::InitPathMode(Path & vPath)
{

	vector<int> iAxis;
	m_NMCServo.GetAxisInfo(m_Axis);
	GetAxis(iAxis);
	vPath.SetAxis(iAxis);

	double xscale, yscale, zscale;
	vPath.GetScale(xscale, yscale, zscale);
	SetScale(xscale, yscale, zscale);
	SetCoordPathMode(vPath.GetPathFreq());
	return 0;
};

int Stage::SetPathFreq(int PathFreq)
{
	m_PathFreq=PathFreq;	
	return 0;
};
int Stage::SetNumberOfPoints(int iNumberOfPoints)
{
	m_BufSize=iNumberOfPoints;
	return 0;
};
int Stage::SetScale(double xScale, double yScale, double zScale)
{
	m_fScale[0] = xScale;
	m_fScale[1] = yScale;
	m_fScale[2] = zScale;
	return 0;
};
int Stage::SetScale()
{
	m_fScale[eXAxis] = m_dEncoderCountsQuad[eXAxis]*m_dLeadScrewTreadsInch[eXAxis]*
						m_dMotorGearRatio[eXAxis]*m_dPulleyRatio[eXAxis];

	m_fScale[eYAxis] = m_dEncoderCountsQuad[eYAxis]*m_dLeadScrewTreadsInch[eYAxis]*
						m_dMotorGearRatio[eYAxis]*m_dPulleyRatio[eYAxis];

	m_fScale[eZAxis] = m_dEncoderCountsQuad[eZAxis]*m_dLeadScrewTreadsInch[eZAxis]*
						m_dMotorGearRatio[eZAxis]*m_dPulleyRatio[eZAxis];
	return 0;
};
int Stage::SetPathAcceleration(double PathAcceleration)
{
	
	m_PathAcceleration=PathAcceleration;
	return 0;
};
int Stage::SetCoordPathMode(int pathFreq)
{
	byte statitems;
	byte ioctrl;
	vector <AxisInfo>::iterator AxisItr;

	if (pathFreq == P_120HZ)    //set fast path mode if using 120 Hz path
	{
		for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
		{
			ioctrl = ServoGetIoCtrl(AxisItr->byAxis);
			ServoSetIoCtrl(AxisItr->byAxis, (byte)(ioctrl | FAST_PATH));
		}

	}
	else    //clear fast path bit if using slower modes
	{
		for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
		{
			ioctrl = ServoGetIoCtrl(AxisItr->byAxis);
			ServoSetIoCtrl(AxisItr->byAxis, (byte)(ioctrl & ~((byte)(FAST_PATH)) ));
		};

	}


//Check that the required status data will be returned with each command:
	statitems = SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX;
	
	for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
	{
		NmcDefineStatus((byte)AxisItr->byAxis, statitems );
		if (  ( NmcGetStatItems( AxisItr->byAxis ) & statitems ) != statitems  )
		{
			ErrorMsgBox("Required status items have not been set");
			return(-1);
		};
	};

	return 0;
}; // Sets the Cooridnated Path mode 
int Stage::InitPathLength(double & fPathLength )
{
	
//	m_Path.InitPathLength(fPathLength);
	return 0;
};	int Stage::InitPathLength(vector<double> & fPathLength )
{
	
//	m_Path.InitPathLength(fPathLength);
	return 0;
};	
int Stage::InitCoordMotion( )
{
	vector <AxisInfo>::iterator AxisItr;
	for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
	{
		if (!ServoStopMotor(AxisItr->byAxis, ServoGetStopCtrl(AxisItr->byAxis) & (byte)AMP_ENABLE)) 
			return(0.0);
	}

	for ( AxisItr=m_Axis.begin() ; AxisItr != m_Axis.end() ; AxisItr++ )
	{
		ServoInitPath(AxisItr->byAxis);   //set the beginning of the path to the current position
	}

	return 0;
};	

int Stage::SetTangentTolerance	( double theta )
{	
//	m_Path.SetTangentTolerance((float)theta);
	return 0;
};
int Stage::ClearSegList		( double x, double y, double z )
{
//	m_Path.ClearSegListA( x, y, z);
	return 0;
};
int Stage::AddStart			( double x, double y, double z )
{
//	m_Path.AddStartA( x, y, z);
	return 0;
};
int Stage::AddLineSeg			( double x, double y, double z )
{
//	m_iRet=m_Path.AddLineSegA( x, y, z);

//	if ( m_iRet == -1)
//	{
//		SimpleMsgBox("Segment added is not tangent to the previous segment");	
//		return eSegNotTangent;
//	}

//	if ( m_iRet == -2)
//	{
//		SimpleMsgBox("Too many segments in the segment list");	
//		return eSegListFull;
//	}
	
	return m_iRet;
};      //end point
int Stage::AddArcSeg			( double x, double y, double z,        //end point
               						  double cx, double cy, double cz,     //center point
               						  double nx, double ny, double nz )
{
//	m_iRet=m_Path.AddArcSegA( x,  y,  z,        //end point
//               				 cx, cy, cz,     //center point
//               				 nx, ny, nz );    //normal

//	if ( m_iRet == -1)
//	{
//		SimpleMsgBox("Segment added is not tangent to the previous segment");
//		return eSegNotTangent;
//	}

//	if ( m_iRet == -2)
//	{
//		SimpleMsgBox("Too many segments in the segment list");
//		return eSegListFull;
//	}

//	if ( m_iRet == -3)
//	{
//		SimpleMsgBox("Values given do not define a correct arc");
//		return eSegInvalidArc;
//	}

	return m_iRet;
};    //normal
int	Stage::AddSplineSeg( vector<double> CtrlPts, int iParts, enSplineType SplineType )
{
	int iRet;
//	iRet=m_Path.AddSplineSegP( CtrlPts, iParts, SplineType );    //normal

	return iRet;
};    //normal

int Stage::SetFeedrate			( double fr)
{
	
//	m_Path.SetFeedrate(float(fr));
	return 0;
};
int Stage::SetOrigin			( double xoffset, double yoffset, double zoffset )
{
	
//	m_Path.SetOrigin( (float)xoffset, (float)yoffset, (float)zoffset);
	return 0;
};
int Stage::SetPathParams		(  )
{
	
//	if ( CheckAxis() != 0 )
//		return eInvalidAxis;

//	if (m_PathFreq == 0)    //set fast path mode if using 120 Hz path
//		return eInvalidPathFreq;
	
//	if (m_BufSize == 0)
//		return eInvalidBufferSize;
	
//	if ( CheckScale() != 0 )
//		return eInvalidScale;

//	if (m_PathAcceleration == 0)
//		return eInvalidAcceleration;

//	m_Path.SetPathParams(m_PathFreq, m_BufSize,
//		                 m_fScale[0], m_fScale[1], m_fScale[2],
//						 m_PathAcceleration);

	SetCoordPathMode( P_60HZ );
	return 0;
};

int Stage::SetPathParams		( int freq, int nbuf,
								  int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
								  float xscale, float yscale, float zscale, 
								  float accel )
{

	return 0;
};


int Stage::OpenPointsOut	(string file)
{
//	m_Path.OpenPointsOut(file);
	return 0;	
};
int Stage::ClosePointsOut()
{
//	m_Path.ClosePointsOut();
	return 0;	
};



eServoError Stage::CheckAxis			()
{
	for (int i=0 ; i <= m_iServoModules-1 ; i++)
	{
		if (m_Axis[i].byAxis == 0 )
			return eInvalidAxis;
	}
	return (eServoError)0;
};
eServoError Stage::CheckScale			()
{
	for (int i=0 ; i <= m_iServoModules-1 ; i++)
	{
		if (m_fScale[i] == 0 )
			return eInvalidScale;
	}
	return (eServoError)0;
};

int Stage::ExecutePath			( )
{
	//
//Initialize the path just before execution
//
//	InitPath();

//
//Download path points to the PIC-SERVO CMC modules until all points are
//  downloaded.  Motion will begin automatically when the minimum number
//  of path points have been loaded.
//
//	while ( AddPathPoints() != -1 ) ;

//
//Poll the X axis module to detect when the path is complete
//
	do
	{
		NmcNoOp(1);   //retrieve current status data
	}
	while ( ServoGetAux(1) & PATH_MODE );   //poll while still in path mode	
	return 0;	
};

int Stage::CreatePathPoints	( vector<PathPoint> & vPath, vector<double> dPathLength )
{
	
	//return (m_Path.CreatePathPointsP(vPath, dPathLength));
	return 0;
};
int Stage::AddPathPoints       ( vector<PathPoint> & vPath )
{
	int  iServoNPoints		=0;
	int  iPointsToAdd		=0;
	long PathX[10], PathY[10], PathZ[10];
	int  iPathPoints =0;
	int  iPointsRead =0;
	byte bStatus;
	vector<PathPoint>::const_iterator  PathItr;
	PathPoint PPoint;

//m_CurrVel = m_CmdMaxVel = 0.0 defines a feedhold condition
//Setting m_CmdMaxVel to a non-zero value will resume path execution
//	m_Path.GetCmdPathSpeed(m_PathSpeed);
	if (m_PathSpeed==0.0) 
		return eFeedRateNotSet;
	
	PathItr = vPath.begin();
	while (PathItr != vPath.end())  // The while loop ends when there are no points in the buffer
	{
		Sleep(500);

  //punt when PIC-SERVO buffer is full
		do
		{
			for ( iPointsToAdd = 0; iPointsToAdd < 7; iPointsToAdd++)
			{
				if (PathItr == vPath.end())
					break;

				PPoint=*PathItr;
				PathX[iPointsToAdd]=PPoint.X;
				PathY[iPointsToAdd]=PPoint.Y; 
				PathZ[iPointsToAdd]=PPoint.Z;
				PathItr++;
				iPathPoints++;
			};
			
			if (!ServoAddPathpoints(m_Axis[0].byAxis, iPointsToAdd, PathX, m_PathFreq)) return(-2);
			if (!ServoAddPathpoints(m_Axis[1].byAxis, iPointsToAdd, PathY, m_PathFreq)) return(-2);
			if (m_Axis[2].byAxis)
			if (!ServoAddPathpoints(m_Axis[2].byAxis, iPointsToAdd, PathZ, m_PathFreq)) return(-2);

			if (!NmcNoOp(m_Axis[0].byAxis)) return(-2);                     //read num points from X
			iServoNPoints=ServoGetNPoints(m_Axis[0].byAxis);
// If the point are not being read then return because an error occurred
			if (iServoNPoints > 87)
				return ePathPointNotRead;
			
		} while ((iServoNPoints < m_BufSize) && iPathPoints < vPath.size());
		
		if (!NmcNoOp(m_Axis[2].byAxis)) 
			return(-2);  //make sure data is updated even if points are not added
			
		if (!NmcNoOp(m_Axis[1].byAxis)) 
			return(-2);
		
		if (!NmcNoOp(m_Axis[0].byAxis)) 
			return(-2);


		bStatus=ServoGetAux(m_Axis[0].byAxis);
		if (!(bStatus & PATH_MODE)) 
		{//start path mode when buffer full
			bStatus=ServoStartPathMode(m_group, m_leader);
			if (!bStatus) 
				return(-2);
		}

	}

	return iPathPoints;	
};
int Stage::AddPathPoints       ( vector<PathPoint> & vPath, int maxPointsToStore, double pathSpeed, int pathFreq )
{
	int  iServoNPoints		=0;
	int  iPointsToAdd		=0;
	long PathX[10], PathY[10], PathZ[10];
	int  iPathPoints =0;
	int  iPointsRead =0;
	byte bStatus;
	vector<PathPoint>::const_iterator  PathItr;
	PathPoint PPoint;

//m_CurrVel = m_CmdMaxVel = 0.0 defines a feedhold condition
//Setting m_CmdMaxVel to a non-zero value will resume path execution
	
	if (pathSpeed==0.0) 
		return eFeedRateNotSet;
	
	PathItr = vPath.begin();
	while (PathItr != vPath.end())  // The while loop ends when there are no points in the buffer
	{
		Sleep(500);

  //punt when PIC-SERVO buffer is full
		do
		{
			for ( iPointsToAdd = 0; iPointsToAdd < 7; iPointsToAdd++)
			{
				if (PathItr == vPath.end())
					break;

				PPoint=*PathItr;
				PathX[iPointsToAdd]=PPoint.X;
				PathY[iPointsToAdd]=PPoint.Y; 
				PathZ[iPointsToAdd]=PPoint.Z;
				PathItr++;
				iPathPoints++;
			};
			
			if (!ServoAddPathpoints(m_Axis[0].byAxis, iPointsToAdd, PathX, pathFreq)) return(-2);
			if (!ServoAddPathpoints(m_Axis[1].byAxis, iPointsToAdd, PathY, pathFreq)) return(-2);
			if (m_Axis[2].byAxis)
			if (!ServoAddPathpoints(m_Axis[2].byAxis, iPointsToAdd, PathZ, pathFreq)) return(-2);

			if (!NmcNoOp(m_Axis[0].byAxis)) return(-2);                     //read num points from X
			iServoNPoints=ServoGetNPoints(m_Axis[0].byAxis);
// If the point are not being read then return because an error occurred
			if (iServoNPoints > 87)
				return ePathPointNotRead;
			
		} while ((iServoNPoints < maxPointsToStore) && iPathPoints < vPath.size());
		
		if (!NmcNoOp(m_Axis[2].byAxis)) 
			return(-2);  //make sure data is updated even if points are not added
			
		if (!NmcNoOp(m_Axis[1].byAxis)) 
			return(-2);
		
		if (!NmcNoOp(m_Axis[0].byAxis)) 
			return(-2);


		bStatus=ServoGetAux(m_Axis[0].byAxis);
		if (!(bStatus & PATH_MODE)) 
		{//start path mode when buffer full
			bStatus=ServoStartPathMode(m_group, m_leader);
			if (!bStatus) 
				return(-2);
		}

	}

	return iPathPoints;	
};
int Stage::InitPath			( eAxis nAxis )
{
	
	ServoInitPath(m_Axis[nAxis].byAxis);
	return 0;	
};	
int Stage::StartPathMode		( int iGroupAddr, int iGroupLeader)
{	
	return ServoStartPathMode((byte)iGroupAddr, (byte)iGroupLeader);	
};	
