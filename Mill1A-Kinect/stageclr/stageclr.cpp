// This is the main DLL file.

#include "stdafx.h"
#include "stageclr.h"
#include <vcclr.h>

using namespace stageclr;
CStage::~CStage()
{
   if (this->_pStage != 0)
   {
      delete this->_pStage;
      this->_pStage = 0;
   }
};
CStage::CStage( String^  COMPORT )
{
	_pStage = Singleton<Stage>::InstancePtr();
	char* str2 = (char*)Marshal::StringToHGlobalAnsi(COMPORT).ToPointer();
	_pStage->Initialize(str2);
	Marshal::FreeHGlobal((IntPtr)str2);

}
int CStage::Initialize	( String^ COMPORT )
{
	char* str2 = (char*)Marshal::StringToHGlobalAnsi(COMPORT).ToPointer();
	_pStage->Initialize	( str2 );
	Marshal::FreeHGlobal((IntPtr)str2);
	return 0;
};

int CStage::SetEncoderCounts(int nAxis, double counts)
{
	_pStage->SetEncoderCounts((eAxis)nAxis, counts);
	return 0;
};
int CStage::SetLeadScrewPitch(int nAxis, double pitch)
{
	_pStage->SetLeadScrewPitch((eAxis)nAxis, pitch);
	return 0;	
};
int CStage::SetMotorGearRatio(int nAxis, double ratio)
{
	_pStage->SetMotorGearRatio((eAxis)nAxis, ratio);
	return 0;
};
int CStage::SetPulleyRatio(int nAxis, double ratio)
{
	_pStage->SetPulleyRatio((eAxis)nAxis, ratio);
	return 0;
};
int CStage::SetGainLimits(int eXAxis, double IntergralLimit, 
						              double OutputLimit,
						              double CurrentLimit,
						              double ErrorLimit,
						              double ServoRate,
						              double DeadBand )
{
	 GainLimit GainLimits;
	 GainLimits.dIntegralLimit	=IntergralLimit; 
	 GainLimits.dOutputLimit	=OutputLimit; 
	 GainLimits.dCurrentLimit	=CurrentLimit; 
	 GainLimits.dErrorLimit		=ErrorLimit; 
	 GainLimits.dServoRate		=ServoRate;
	 GainLimits.dDeadBand		=DeadBand;
	
	_pStage->SetGainLimits((eAxis)eXAxis, GainLimits);
	return 0;
};
bool CStage::IsInMotion		    (  )
{

	return _pStage->IsInMotion();
};

int CStage::SetGain		    ( int nAxis,
		                      int siProportional, 
						  	  int siDifferential, 
							  int siIntegral )
{

	_pStage->SetGain( (eAxis)nAxis,
		              siProportional, 
					  siDifferential, 
				      siIntegral );
	return 0;
};

int CStage::SetVel		 ( double lXVel, double lYVel, double lZVel )
{
	_pStage->SetVel( lXVel, lYVel, lZVel );
	return 0;
};
int CStage::SetAccel	 ( double lXAcc, double lYAcc, double lZAcc)
{
	_pStage->SetAccel( lXAcc, lYAcc, lZAcc);
	return 0;
};
int CStage::EnableAmp	 ()
{
	_pStage->EnableAmp();
	return 0;
};
int CStage::ResetPos	 ()
{
	_pStage->ResetPos();
	return 0;
};
int CStage::MoveTo		 ( double XPos, double YPos, double ZPos )
{
	_pStage->MoveTo( XPos, YPos, ZPos);
	return 0;
};
int CStage::MoveTo		 ( double XPos, double YPos, double ZPos, bool WithWait )
{
	_pStage->MoveTo( XPos, YPos, ZPos, WithWait);
	return 0;
};
int CStage::MoveRel		 ( double XPos, double YPos, double ZPos )
{
	_pStage->MoveRel( XPos, YPos, ZPos);
	return 0;
	
};
int CStage::MoveRel		 ( double XPos, double YPos, double ZPos, bool WithWait )
{
	_pStage->MoveRel( XPos, YPos, ZPos,WithWait);
	return 0;
	
};
int CStage::Stop		 ()
{
	_pStage->StopMotor();
	return 0;
};

int CStage::GetPos		 ( double & XPos, double & YPos, double & ZPos)
{
	_pStage->GetPos( XPos, YPos, ZPos);
	return 0;
};
int CStage::GetPos( [Out]double %XPos, [Out]double %YPos, [Out]double %ZPos)
{
	double X, Y, Z;

	_pStage->GetPos( X, Y, Z);
	XPos=X;
	YPos=Y;
	ZPos=Z;
	return 0;
};
int CStage::GetCmdVel		 ( [Out]double % lXVel, [Out]double % lYVel, [Out]double % lZVel )
{
	double XVel, YVel, ZVel;

	_pStage->GetCmdVel( XVel, YVel, ZVel );

	lXVel=XVel;
	lYVel=YVel;
	lZVel=ZVel;
	return 0;	
};

int CStage::GetCmdAccel	 ( [Out]double % XAccel, [Out]double % YAccel, [Out]double % ZAccel )
{
	double XAcc, YAcc, ZAcc;

	_pStage->GetCmdAccel(XAcc, YAcc, ZAcc);
	XAccel=XAcc;
	YAccel=YAcc;
	ZAccel=ZAcc;
	return 0;
};
int CStage::Rotate		 ( double Angle )
{
	_pStage->Rotate(Angle);	
	return 0;	
};
Stage* CStage::GetNativePtr()
{
	 return _pStage;
};

int	CStage::IsCoordMotionComplete()
{
	_pStage->IsCoordMotionComplete();
	return 0;
};
int	CStage::ExecuteCoordMotion()
{
	_pStage->ExecuteCoordMotion();
	return 0;
};
int CStage::ClearCoordMotion()
{

	_pStage->ClearCoordMotion();
	return 0;
};
int	CStage::SetGroupAddress(int group, int leader)
{
	_pStage->SetGroupAddress(group, (eAxis)leader);
	return 0;
};
int	CStage::StopResetMotors()
{
	_pStage->StopResetMotors();
	return 0;
};
int	CStage::SetPathStatus()
{
	_pStage->SetPathStatus();
	return 0;
};
int	CStage::SetPathFreq(int PathFreq)
{
	_pStage->SetPathFreq(PathFreq);
	return 0;
};
int	CStage::SetNumberOfPoints(int iNumberOfPoints)
{
	_pStage->SetNumberOfPoints(iNumberOfPoints);
	return 0;
};
int	CStage::SetScale(double xScale, double yScale, double zScale)
{
	_pStage->SetScale(xScale, yScale, zScale);
	return 0;
};
int	CStage::SetScale()
{
	_pStage->SetScale();
	return 0;
};
int	CStage::SetPathAcceleration(double Acceleration)
{
	_pStage->SetPathAcceleration(Acceleration);
	return 0;
};
int	CStage::SetTangentTolerance	( double theta )
{
	_pStage->SetTangentTolerance	( theta );
	return 0;
};
int	CStage::ClearSegList		( double x, double y, double z )
{
	_pStage->ClearSegList		( x, y, z );
	return 0;
};
int	CStage::AddStart			( double x, double y, double z )
{
	_pStage->AddStart			( x, y, z );
	return 0;
};
int	CStage::AddLineSeg			( double x, double y, double z )        //end point
{
	_pStage->AddLineSeg			( x, y, z );  
	return 0;
};    
int	CStage::AddArcSeg			( double x, double y, double z,        //end point
               					  double cx, double cy, double cz,     //center point
               					  double nx, double ny, double nz )    //normal
{
	_pStage->AddArcSeg			( x, y, z,        //end point
               					  cx, cy, cz,     //center point
               					  nx, ny, nz );
	return 0;
};  
int	CStage::AddSplineSeg( vector<double> CtrlPts, int iParts, enSplineType SplineType ) //normal
{
	_pStage->AddSplineSeg( CtrlPts, iParts, SplineType );
	return 0;	
};
int	CStage::SetFeedrate			( double fr)
{
	_pStage->SetFeedrate( fr);
	return 0;	
};
int	CStage::SetOrigin			( double xoffset, double yoffset, double zoffset )
{
	
	_pStage->SetOrigin	( xoffset, yoffset, zoffset );
	return 0;	
};
int	CStage::SetPathParams		(  )
{
	_pStage->SetPathParams	(  );
	return 0;	
};

int	CStage::SetPathParams	( int freq, int nbuf,
							  int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
							  float xscale, float yscale, float zscale, 
							  float accel )
{
	_pStage->SetPathParams	( freq, nbuf,
							  xaxis, yaxis, zaxis, groupaddr, leaderaddr,
							  xscale, yscale, zscale, 
							  accel );
	return 0;	
};

int	CStage::OpenInFile		(String^ file)
{
	return 0;
};
int	CStage::CloseInFile		()
{
	
	return 0;
};
int	CStage::OpenOutFile		(String^ file)
{
	
	return 0;
};
int	CStage::CloseOutFile	()
{

	return 0;
};
int	CStage::OpenPointsOut	(String^ file)
{
	return 0;
};
int	CStage::ClosePointsOut()
{
	return 0;
};
int	CStage::ReadInFile		()
{
	return 0;
};
int	CStage::ReadInFileSimulation()
{
	return 0;
	
};

