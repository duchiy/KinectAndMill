// stageclr.h
#include "Stage.h"

using namespace System;
using namespace System::Runtime::InteropServices;
#pragma make_public(Stage);
namespace stageclr {


	public ref class CStage
	{
		public:
			virtual ~CStage();
			CStage() : _pStage() { }
			CStage( String^ ComPort );
			int	Initialize				( String^ ComPort );
			int SetEncoderCounts(int nAxis, double counts);
			int SetLeadScrewPitch(int nAxis, double pitch);
			int SetMotorGearRatio(int nAxis, double ratio);
			int SetPulleyRatio(int nAxis, double ratio);
			int SetGainLimits(int eXAxis,	double IntergralLimit, 
											double OutputLimit,
											double CurrentLimit,
											double ErrorLimit,
											double ServoRate,
											double DeadBand);
			bool IsInMotion		 ( );

			int SetGain		 ( int nAxis,
		                      int siProportional, 
						  	  int siDifferential, 
							  int siIntegral );
			int SetVel		 ( double lXVel, double lYVel, double lZVel );
			int SetAccel	 ( double lXAcc, double lYAcc, double lZAcc);
			int EnableAmp	 ();
			int ResetPos	 ();
			int MoveTo		 ( double XPos, double YPos, double ZPos );
			int MoveTo		 ( double XPos, double YPos, double ZPos, bool bWait );
			int MoveRel		 ( double XPos, double YPos, double ZPos );
			int MoveRel		 ( double XPos, double YPos, double ZPos, bool bWait );
			int Stop		 ();
			int GetPos		 ( double & XPos, double & YPos, double & ZPos);
			int GetPos		 ( [Out]double % XPos, [Out]double % YPos, [Out]double % ZPos);
			int GetCmdVel	 ( [Out]double % lXVel, [Out]double % lYVel, [Out]double % lZVel );
			int GetCmdAccel	 ( [Out]double % XAccel, [Out]double % YAccel, [Out]double % ZAccel );
			int Rotate		 ( double Angle );
			Stage* GetNativePtr();

// Methods for Coordinated motion
			int	IsCoordMotionComplete();
			int	ExecuteCoordMotion();
			int ClearCoordMotion();
			int	SetGroupAddress(int group, int leader);
			int	StopResetMotors();
			int	SetPathStatus();
			int	SetPathFreq(int PathFreq);
			int	SetNumberOfPoints(int iNumberOfPoints);
			int	SetScale(double xScale, double yScale, double zScale);
			int	SetScale();
			int	SetPathAcceleration(double Acceleration);

			int	SetTangentTolerance	( double theta );
			int	ClearSegList		( double x, double y, double z );
			int	AddStart			( double x, double y, double z );
			int	AddLineSeg			( double x, double y, double z );      //end point
			int	AddArcSeg			( double x, double y, double z,        //end point
               						  double cx, double cy, double cz,     //center point
               						  double nx, double ny, double nz );   //normal
			int	AddSplineSeg( vector<double> CtrlPts, int iParts, enSplineType SplineType );    //normal
			int	SetFeedrate			( double fr);
			int	SetOrigin			( double xoffset, double yoffset, double zoffset );
			int	SetPathParams		(  );

			int	SetPathParams		( int freq, int nbuf,
									  int xaxis, int yaxis, int zaxis, int groupaddr, int leaderaddr,
									float xscale, float yscale, float zscale, 
									float accel );
	
			int		OpenInFile		(String^ file);
			int		CloseInFile		();
			int		OpenOutFile		(String^ file);
			int		CloseOutFile	();
			int		OpenPointsOut	(String^ file);
			int		ClosePointsOut();
	
			int		ReadInFile		();
			int		ReadInFileSimulation();

		// TODO: Add your methods for this class here.
		private:
			Stage* _pStage;
	};

}
