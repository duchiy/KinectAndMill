//---------------------------------------------------------------------------

#include "stdafx.h"
#include <math.h>
#include "path.h"
#include "picservo.h"
#include "Point.h"
#include <list>
using namespace std;
//using namespace vLinePath;


//---------------------------------------------------------------------------
//Globals:

//---------------------------------------------------------------------------
//  Geometric functions
//---------------------------------------------------------------------------
//Returns the magnitude of a floating point vector
Path::Path()
{
	m_TangentTol = TAN_3DEGREE;    //minimum cos(th) for angle between tangents
	m_PathFreq = P_30HZ;		//selected path frequency
	m_x = 1;  				//axes addresses
	m_y = 2;
	m_z = 3;
//	m_group = 0xFF;			//m_group address for coordinated controllers
//	m_leader = 0x00;			//m_group m_leader address for coordinated controllers
	m_group =(byte)128;
	m_leader = 0x01;			//m_group m_leader address for coordinated controllers
};

Path::~Path()
{
};

int	Path::OpenPointsOut(string file)
{
	m_fpoints.open ( file.c_str(), ios::out | ios::trunc  );
	if (!m_fpoints.is_open())
		return -1;
	
	return 0;	return 0;
};
int	Path::ClosePointsOut()
{
	if (m_fpoints.is_open())
		m_fpoints.close();

	return 0;
};

int	Path::SetAxis( vector<int> iAxis )
{
	m_x=(byte)iAxis[0];
	m_y=(byte)iAxis[1];
	m_z=(byte)iAxis[2];
	return 0;
};

int	Path::GetCmdPathSpeed(double & Speed)
{
	Speed=m_CmdMaxVel;
	return 0;
};
int	Path::GetPathFreq(int & pathFreq)
{
	pathFreq = m_PathFreq;
	
	return 0;
};
int	Path::GetPathFreq()
{	
	return m_PathFreq;
};
int	Path::GetMaxPointsToStore(int & maxPoints)
{
	maxPoints = m_BufSize;
	return 0;	
};
int	Path::GetScale(double & xscale, double & yscale, double & zscale)
{
	xscale =m_UnitsToCountsX;
	yscale =m_UnitsToCountsY;
	zscale =m_UnitsToCountsZ;
	return 0;
}

//---------------------------------------------------------------------------
// Coordinate frame inversion: *B = inverse(A)

double	Path::mag( vector<double> Pt )
{
	double dmag=0.0;
//	if ( Pt.size() == 0)

	for (int i=0 ; i <= Pt.size()-1 ; i++)
		dmag = dmag + Pt[i]*Pt[i];

	dmag = sqrt(dmag);
	return dmag;
};
double	Path::dot( vector<double> Pt1, vector<double> Pt2 )
{
	double ddot = 0.0;
//	if ( ( Pt1.size() == 0) || (Pt2.size() == 0) )
//		return -1;

//	if ( Pt1.size() != Pt2.size())
//		return -2;

	for (int i=0 ; i <= Pt1.size()-1 ; i++)
		ddot = ddot + Pt1[i]*Pt2[i];

	return ddot;
};
int Path::normalize( vector<double> & Pt )
{
	if ( Pt.size() == 0)
		return -1;
	
	double dMag;
	int iError;
	dMag=mag( Pt );
	if ( dMag < 0)
		return -3;

	for (int i=0 ; i <= Pt.size()-1 ; i++)
		Pt[i]=Pt[i]/dMag;
	
	
	return 0;
};
vector<double> Path::scale( double dScale, vector<double>  Pt )
{

//	if ( Pt.size() == 0)
//		return -1;

	for (int i=0 ; i <= Pt.size()-1 ; i++)
		Pt[i]=Pt[i]*dScale;

	return Pt;
};
vector<double>	Path::cross	( vector<double> Pt1, vector<double> Pt2 )
{
	vector<double> Pt3(3);
//	if ( ( Pt1.size() == 0) || (Pt2.size() == 0) )
//		return -1;

//	if ( Pt1.size() != Pt2.size())
//		return -2;

	Pt3[0] = Pt1[1]*Pt2[2] - Pt1[2]*Pt2[1];
	Pt3[1] = Pt1[2]*Pt2[0] - Pt1[0]*Pt2[2];
	Pt3[2] = Pt1[0]*Pt2[1] - Pt1[1]*Pt2[0];

	return Pt3;
};      // x=x cross y
vector<double>	Path::subtract( vector<double> Pt1, vector<double> Pt2 )
{
	vector<double> Pt3(3);

//	if ( ( Pt1.size() == 0) || (Pt2.size() == 0) )
//		return -1;

//	if ( Pt1.size() != Pt2.size())
//		return -2;

	Pt3[0]=Pt1[0] - Pt2[0];
	Pt3[1]=Pt1[1] - Pt2[1];
	Pt3[2]=Pt1[2] - Pt2[2];

	return Pt3;
};  // z=x-y
vector<double> Path::fvmult(vFrame F, vector<double> Pt )
{
	vector<double> retPt(3);
	retPt[0] = F.PtX[0]*Pt[0] + F.PtY[0]*Pt[1] + F.PtZ[0]*Pt[2]  + F.PtP[0];
	retPt[1] = F.PtX[1]*Pt[0] + F.PtY[1]*Pt[1] + F.PtZ[1]*Pt[2]  + F.PtP[1];
	retPt[2] = F.PtX[2]*Pt[0] + F.PtY[2]*Pt[1] + F.PtZ[2]*Pt[2]  + F.PtP[2];
	return retPt;
	
		
};


//void	Path::finvert(frame A, frame *B)
//{
		
//};

int	Path::GetTanVectP(vPathSegment *s, vector<double> & PtP, bool bEndPt)
{
	vector<double> PtQ(3);

	if (s->m_segType == enLine)
	{
//		vLinePath::vLine LineSeg;
//		LineSeg=*(vLinePath::vLine *)s;
		vLine LineSeg;
		LineSeg=*(vLine *)s;

		PtP = subtract(LineSeg.m_End, LineSeg.m_Start);

		if (mag(PtP) < m_Tol) 
			return(-1);
		else 
			return(0);
	}
	else if (s->m_segType == enArc)
	{
		vArc ArcSeg;
		ArcSeg=*(vArc *)s;
		if (bEndPt )
		{
			PtQ=subtract(ArcSeg.m_Start, ArcSeg.m_Center);
		}
		else
		{
			PtQ=subtract(ArcSeg.m_End, ArcSeg.m_Center);
		}

		if (mag(PtQ) < m_Tol) 
			return(-1);

		normalize(PtQ);
		PtP=cross(ArcSeg.m_Normal, PtQ);

		return(0);
	}

	return(0);
};

//---------------------------------------------------------------------------
// PathSegment list functions
//---------------------------------------------------------------------------
//Theta = allowable angle (in degrees) between continuous path segments
void Path::SetTangentTolerance(float theta)
{
	m_TangentTol = cos(theta*DTOR);
}
//---------------------------------------------------------------------------
//Extract the reference frame for an arc
//Also fills in the radius and arclength

//---------------------------------------------------------------------------
//Returns a point p which lies on a line segment, and is a distance s from
//the start of the line segment
//---------------------------------------------------------------------------
//Extract the reference frame for an arc
//Also fills in the radius and arclength

void Path::GetArcFrameP(vArc *ArcSeg, vFrame *F)
{
	float q, theta;
	vector<double> Pt(3);
	vector<double> PtX(3);

	if (ArcSeg->m_segType != enArc) return;   //punt if not an arc

	F->PtP = ArcSeg->m_Center;		//origin is at the center
	F->PtX = subtract(ArcSeg->m_Start, ArcSeg->m_Center);
	ArcSeg->m_fRadius =  mag(F->PtX);//extract radius 
	normalize(F->PtX);  // normalize
	q = dot(F->PtX, ArcSeg->m_Normal);     //make sure normal vector is perp. to X
	PtX = scale(q,F->PtX);
	F->PtZ = subtract(ArcSeg->m_Normal,PtX);
	normalize(F->PtZ);
	F->PtY = cross(F->PtZ, F->PtX);

	Pt=subtract(ArcSeg->m_End, ArcSeg->m_Center);

	theta = atan2( dot(Pt,F->PtY), dot(Pt,F->PtX) );
	if (theta < 0.0) theta = TWOPI + theta;
	if (fabs(theta) < 0.001) theta = TWOPI - theta;
	ArcSeg->m_fLength = fabs(ArcSeg->m_fRadius * theta);
}


void Path::ClearSegListA(double xi, double yi, double zi)
{
	vPathSegment * pSeg = new vPathSegment();
	m_vsegList.clear();

	pSeg->SetStart((float)xi, (float)yi, (float)zi);
	m_vsegList.push_back(pSeg);

}

int	Path::AddStartA(double xi, double yi, double zi)
{
	vPathSegment * pSeg = new vPathSegment();
	pSeg->SetStart((float)xi, (float)yi, (float)zi);
	m_vsegList.push_back(pSeg);
	return 0;
};

//---------------------------------------------------------------------------
//Add a line segment to the segment list
//Returns: position in segment list if OK
//         -1 if segment is not tangent
//         -2 if segment list is full
//Function assumes the normal vector of any previous arc segment is accurate


int Path::AddLineSegA(double x, double y, double z)
{
	list<vPathSegment*>::iterator  segListItr;
	vector<double> Start(3), End(3);
	vector<double> pn(3), qn(3);

	if (m_vsegList.size() >= MAXSEG) return(-2);
	End[0]=x;  End[1]=y;  End[2]=z;

	vPathSegment *psPrev = new vPathSegment();
//	vLinePath::vLine *psLine = new vLinePath::vLine();
	vLine *psLine = new vLine();

	psLine->SetEnd(End);

	if (m_vsegList.size() <= 1)
	{
		segListItr=m_vsegList.begin();
		psPrev=*segListItr;
	}
	else
	{
		segListItr=m_vsegList.end();
		segListItr--;
		psPrev=*segListItr;
	}

	if (psPrev->m_segType == enOrigin)
	{
	    float XOrigin, YOrigin, ZOrigin;
		psPrev->GetStart(XOrigin, YOrigin, ZOrigin);
		psLine->SetStart(XOrigin, YOrigin, ZOrigin);

	}
	else    //match start point to end point of prev segment
	{
		float prevX, prevY, prevZ;
		psPrev->GetEnd(prevX, prevY, prevZ);
		psLine->SetStart(prevX, prevY, prevZ);

	}

	pn=subtract(End, psLine->m_Start);
	psLine->SetLength(mag(pn));
	normalize(pn);
//Check tangency with prev. segment for segments > m_Tol:
	if ( (psLine->m_fLength > m_Tol) && (m_vsegList.size() > 1) )
		if ( GetTanVectP( psPrev, qn, false) == 0 )
			if ((dot(pn,qn) < m_TangentTol) && (psPrev->m_segType != enOrigin))
				return(-1);

	m_vsegList.push_back(psLine);
	vLine myLine = *psLine;
	return(m_vsegList.size() - 1);
}

//---------------------------------------------------------------------------
//Add an arc segment to the segment list
//Returns: position in segment list if OK
//         -1 if segment is not tangent
//         -2 if segment list is full
//		   -3 if arc data invalid
//(Invalid arc data - zero fLength. normal, radius < m_Tol, normal not perp.)
//Function assumes the normal vector of any previous arc segment is accurate

int Path::AddArcSegA(	double x, double y, double z,        //end point
						double cx, double cy, double cz,     //center point
						double nx, double ny, double nz )    //normal
{
	vFrame Fr;
	list<vPathSegment*>::iterator  segListItr;
	vector<double> pn(3);
	vector<double> qn(3);
	vector<double> end(3), center(3);

	if (m_vsegList.size() >= MAXSEG) return(-2);
	
	end[0]    = x;  end[1]    = y;   end[2]   = z;
	center[0] = cx; center[1] = cy; center[2] = cz;

	vPathSegment * psPrev = new vPathSegment();
	vArc * psArc = new vArc();

	psArc->SetEnd(x, y, z);
	psArc->SetCenter(cx, cy, cz);
	psArc->SetNormal(nx, ny, nz);

	if (m_vsegList.size() <= 1)
	{
		segListItr=m_vsegList.begin();
		psPrev=*segListItr;
	}
	else
	{
		segListItr=m_vsegList.end();
		segListItr--;
		psPrev=*segListItr;
	}

	if (psPrev->m_segType == enOrigin)
	{
	    float XOrigin, YOrigin, ZOrigin;
		psPrev->GetStart(XOrigin, YOrigin, ZOrigin);
		psArc->SetStart(XOrigin, YOrigin, ZOrigin);

	}
	else
	{
	    float prevX, prevY, prevZ;
		psPrev->GetEnd(prevX, prevY, prevZ);
		psArc->SetStart(prevX, prevY, prevZ);

	}


//Normalize n and punt if too small
	if ( mag(psArc->m_Normal) < m_Tol )
		return(-3);

	normalize(psArc->m_Normal);
//Find radius to End, and punt if too small
	pn=subtract(end,center);
	psArc->SetRadius(mag(pn));

	if ( psArc->m_fRadius < m_Tol) return(-3);

//Check if normal is perp to Center->End vector and punt if not
	if ( fabs(dot(psArc->m_Normal, pn)) > 0.001 ) return(-3);

//Find radius to Start, and punt if not equal to radius to End
	pn=subtract(psArc->m_Start,center);
	if ( fabs(psArc->m_fRadius - mag(pn)) > m_Tol) return(-3);

//Check if normal is perp to Center->End vector and punt if not
	if ( fabs(dot(psArc->m_Normal, pn)) > 0.001 ) return(-3);

//Check for tangency with prev segment
	if ( m_vsegList.size() > 1 )
	{
		GetTanVectP( psArc, pn, true);    //get current tangent
		if ( GetTanVectP( psPrev, qn, false) == 0 )   //get prev tangent
		if (dot(pn,qn) < m_TangentTol) return(-1);
	}

	GetArcFrameP(psArc, &Fr);  //fills in segment length
	m_vsegList.push_back(psArc);

	vArc myArc = *psArc;

	return(m_vsegList.size() - 1);
}

int	Path::AddSplineSegP( vector<double> CtrlPts, int iParts, enSplineType SplineType )
{
	if ( CtrlPts.size() < 6)
		return -1;
	

	list<vPathSegment*>::iterator  segListItr;
	vector<double> Start(3), End(3);

	int iSpline=0;
	iSpline=CtrlPts.size();
	if (m_vsegList.size() >= MAXSEG) return(-2);

	End[0]=CtrlPts[iSpline-3];  
	End[1]=CtrlPts[iSpline-2];  
	End[2]=CtrlPts[iSpline-1];

	vPathSegment *psPrev = new vPathSegment();
	vSpline * psSpline = new vSpline(CtrlPts, iParts);
	psSpline->m_Type = SplineType;
	psSpline->SetEnd(End);

	if (m_vsegList.size() <= 1)
	{
		segListItr=m_vsegList.begin();
		psPrev=*segListItr;
	}
	else
	{
		segListItr=m_vsegList.end();
		segListItr--;
		psPrev=*segListItr;
	}

	if (psPrev->m_segType == enOrigin)
	{
	    float XOrigin, YOrigin, ZOrigin;
		psPrev->GetStart(XOrigin, YOrigin, ZOrigin);
		psSpline->SetStart(XOrigin, YOrigin, ZOrigin);

	}
	else    //match start point to end point of prev segment
	{
		float prevX, prevY, prevZ;
		psPrev->GetEnd(prevX, prevY, prevZ);
		psSpline->SetStart(prevX, prevY, prevZ);

	}

// Set length
	vector<double> newCtrlPts;
	vector<double> vSpline;
	SplineFactory SplFactory;
	SplFactory.SetType(SplineType);
	vSpline=SplFactory.createSpline(CtrlPts, iParts);
	psSpline->SetLength(vSpline);
	m_vsegList.push_back(psSpline);
	return(m_vsegList.size() - 1);
};    //normal

//---------------------------------------------------------------------------
//Returns a point p which lies on a line segment, and is a distance s from
//the start of the line segment


//vector<double> Path::GetPointOnLine(vLinePath::vLine Line, double fLength)
vector<double> Path::GetPointOnLine(vLine Line, double fLength)
{
	double fT;
	vector<double> Pt(3);

	fT = fLength/Line.m_fLength;
	Pt[0] = Line.m_Start[0] + fT*(Line.m_End[0] - Line.m_Start[0]);
	Pt[1] = Line.m_Start[1] + fT*(Line.m_End[1] - Line.m_Start[1]);
	Pt[2] = Line.m_Start[2] + fT*(Line.m_End[2] - Line.m_Start[2]);
	return Pt;
}

vector<double> Path::GetPointOnArc(vArc Arc, double fLength)
{
	vector<double> Pt(3);
	Pt[0]  = Arc.m_fRadius * cos(fLength/Arc.m_fRadius);
	Pt[1]  = Arc.m_fRadius * sin(fLength/Arc.m_fRadius);
	Pt[2]  = 0;
	Pt=fvmult(m_vCurrArcFrame1, Pt);	
	return Pt;
};
vector<double> Path::GetPointOnSpline( vSpline Spline, double fLength )
{
	vector<double> Pt(3);
	vector<double> ReturnPt;
	// initialize spline
	if (!m_SplineFactory.IsInitialized())
	{
		m_SplineFactory.initialize(Spline.m_Type, Spline.m_CtrlPts , Spline.m_Length, 120);
	}

	m_SplineFactory.GetPoint(fLength, Pt);
	ReturnPt.push_back(Pt[0]);
	ReturnPt.push_back(Pt[1]);
	ReturnPt.push_back(Pt[2]);

	return ReturnPt;
};
vector<double> Path::GetPointOnSplineA( vSpline Spline, double fLength )
{
	vector<double> Pt(3);
	vector<double> ReturnPt;
	// initialize spline
	if (!m_SplineFactory.IsInitialized())
	{
		m_SplineFactory.initialize(Spline.m_Type, Spline.m_CtrlPts , Spline.m_Length, 120);
		m_SplineFactory.GetPoint(0.0, Pt);
		ReturnPt.push_back(Pt[0]);
		ReturnPt.push_back(Pt[1]);
		ReturnPt.push_back(Pt[2]);
		m_SplineFactory.GetPoint(fLength, Pt);
	}
	else
		m_SplineFactory.GetPoint(fLength, Pt);

	ReturnPt.push_back(Pt[0]);
	ReturnPt.push_back(Pt[1]);
	ReturnPt.push_back(Pt[2]);

	return ReturnPt;
};

//---------------------------------------------------------------------------
//Set feedrate in units per second

void Path::SetFeedrate(double fr)
{
	switch (m_PathFreq)       //calculate velocity in units per tick
		{
		case P_30HZ: m_CmdMaxVel = fr/30.0;
    			 break;
		case P_60HZ: m_CmdMaxVel = fr/60.0;
    			 break;
		case P_120HZ: m_CmdMaxVel = fr/120.0;
    			 break;
		default: m_CmdMaxVel = fr/30.0;
		}
}
//---------------------------------------------------------------------------
//Set the origin to which all segment data is relative
void Path::SetOrigin(double xoffset, double yoffset, double zoffset)
{
	m_XOffset = xoffset;  		//origin offset
	m_YOffset = yoffset;
	m_ZOffset = zoffset;
}

int Path::SetPathParams(int freq, int nbuf,
                        double xscale, double yscale, double zscale,
                        double accel )
{

//SimpleMsgBox(" PIC-SERVO Coordinated Control Example\n\n- FOR EVALUATION PURPOSES ONLY -");

	m_PathFreq = freq;        //set to 30 or 60 hz
	m_BufSize = nbuf;   	  	//max num points to store in the PIC-SERVO buffer

	if (fabs(xscale)<1.0 || fabs(yscale)<1.0) return(-2);
	if (fabs(zscale)<1.0) return(-2);

	m_UnitsToCountsX = xscale;   		//Units To X counts
	m_UnitsToCountsY = yscale;
	m_UnitsToCountsZ = zscale;

//Set the m_Tol equivalent to 40 counts of the lowest resolution axis
	m_Tol = fabs(40.0/xscale);
	if (m_Tol < fabs(40.0/yscale)) m_Tol = fabs(40.0/yscale);
	if (m_Tol < fabs(40.0/zscale)) m_Tol = fabs(40.0/zscale);

	switch (m_PathFreq)       //calculate acceleration in units per tick^2
	{
		case P_30HZ: m_CmdAcc = accel/30.0/30.0;
    			 break;
		case P_60HZ: m_CmdAcc = accel/60.0/60.0;
    			 break;
		case P_120HZ: m_CmdAcc = accel/120.0/120.0;
    			 break;
		default: m_CmdAcc = accel/30.0/30.0;
    }


	return(0);
}

//---------------------------------------------------------------------------
//Initializes the coordinated path after all of the segments have been added.
//This function should be called just before the application starts calling
//the function AddPathPoints().
//Returns the overall path length for all of the segments.
//Returns 0.0 on communications error



double Path::InitPathLength(double & CmdPath )
{


	if (m_vsegList.size() == 0)
		return -1;

	list<vPathSegment*>::iterator  segListItr;
	vPathSegment PathSeg;
	int i;

	double dCmdPath = 0.0;

	segListItr=m_vsegList.begin();
	PathSeg=*segListItr;
	if (PathSeg.m_segType != enOrigin)
		return enNoOrigin;

	segListItr++;
	for (i=0; i<m_vsegList.size()-1; i++) 
	{
		PathSeg=*segListItr;
		if (PathSeg.m_segType == enArc)
		{
			vArc ArcSeg;
			ArcSeg=*(vArc*)*segListItr;
			GetArcFrameP( &ArcSeg, &m_vCurrArcFrame1 );
			dCmdPath = dCmdPath + ArcSeg.m_fLength;

		};
		if (PathSeg.m_segType == enLine)
		{
			dCmdPath = dCmdPath + PathSeg.m_fLength;
		};

		if (PathSeg.m_segType == enSpline)
		{
			m_SplineFactory.uninitialize();
			vSpline SplineSeg;
			SplineSeg=*(vSpline*)*segListItr;
			dCmdPath = dCmdPath + SplineSeg.m_fLength;
		};
		segListItr++;
	}
	
	CmdPath=dCmdPath;
//make sure we exit path mode first

	return 0;
}
int	Path::InitPathLength(vector<double> & CmdPath )
{
	vector<double> dLength;
	if (m_vsegList.size() == 0)
		return -1;

	list<vPathSegment*>::iterator  segListItr;
	vPathSegment PathSeg;
	int i;

	double dCmdPath = 0.0;

	segListItr=m_vsegList.begin();
	PathSeg=*segListItr;
	if (PathSeg.m_segType != enOrigin)
		return enNoOrigin;

	segListItr++;
	for (i=0; i<m_vsegList.size()-1; i++) 
	{
		PathSeg=*segListItr;
		if (PathSeg.m_segType == enArc)
		{
			vArc ArcSeg;
			ArcSeg=*(vArc*)*segListItr;
			GetArcFrameP( &ArcSeg, &m_vCurrArcFrame1 );
			dCmdPath = dCmdPath + ArcSeg.m_fLength;

		};
		if (PathSeg.m_segType == enLine)
		{
			dCmdPath = dCmdPath + PathSeg.m_fLength;
		};

		if (PathSeg.m_segType == enOrigin)
		{
			CmdPath.push_back(dCmdPath);
			dCmdPath = 0.0;
		};

		if (PathSeg.m_segType == enSpline)
		{
			m_SplineFactory.uninitialize();
			vSpline SplineSeg;
			SplineSeg=*(vSpline*)*segListItr;

//			if (SplineSeg.m_Type == enCubic)
//			{
//				int iSize=SplineSeg.m_Length.size();
//				SplineSeg.m_Length[1]=SplineSeg.m_Length[0]+SplineSeg.m_Length[1];
//				SplineSeg.m_Length[0]=0.0;
//				iSize--;
//				SplineSeg.m_Length[iSize-1]=SplineSeg.m_Length[iSize-1]+SplineSeg.m_Length[iSize];
//				SplineSeg.m_Length[iSize]=0.0;
//			
//			}
			
			for (int i=0; i <= SplineSeg.m_Length.size()-1; i++)
				CmdPath.push_back(SplineSeg.m_Length[i]);

			return 0;
		};
		segListItr++;
	}
	
	CmdPath.push_back(dCmdPath);

	return 0;
};

//---------------------------------------------------------------------------
//Gets the next point in the path.
//returns enEndOfPath if already at the end of the path
//returns enLastPointOfPath if the last point in the path
//returns Current PathSegment Number otherwise 
//float CmdPath - total path length

int Path::GetPathPointsA(vector<PathPoint> & vPath, double CmdPath)
{
		//float CmdPath - total path length

	vPathSegment PathSeg;
	enSegMessage enReturn=enNormalPathRet;
	
	double		CurrSeg		= 0.0;  //current segment number
	double		CurrPath	= 0.0;  //total length of path already downloaded
	double		SegPath		= 0.0;  //length of path in current segment
	double		CurrVel		= 0.0;  //Current Velocity
	bool		bFinalDecel	= false;

	bool				bNewSegment	=false;
	vector<PathPoint>	CompPt;  
	vLine	Line;
	vArc		Arc;

	while (CurrPath <= CmdPath)  //GetNextPathpoint() sets the global at_end
	{

//First check if decelerating to the endpoint
		if (bFinalDecel || (CurrVel*CurrVel > 2*m_CmdAcc*(CmdPath - CurrPath)) )
		{
			CurrVel -= m_CmdAcc;
			if (CurrVel<m_CmdAcc) CurrVel = m_CmdAcc;   //use m_CmdAcc value as minimum velocity
			bFinalDecel = true;
		}
		else if (CurrVel<m_CmdMaxVel)      //check for acceleration to current m_CmdMaxVel
		{
			CurrVel+=m_CmdAcc;
			if (CurrVel>m_CmdMaxVel) CurrVel = m_CmdMaxVel;
		}
		else if (CurrVel>m_CmdMaxVel)      //check for deceleration to current m_CmdMaxVel
		{
			CurrVel-=m_CmdAcc;
			if (CurrVel<m_CmdMaxVel) CurrVel = m_CmdMaxVel;
		}

	
		PathSeg=*m_vsegListItr;
		if (PathSeg.m_segType == enOrigin)
		{
			m_vsegListItr++;  						 	// & move to next segment
			break;
		}

		if ( (SegPath + CurrVel) > PathSeg.m_fLength )   //if past end of segment
		{
			SegPath -= PathSeg.m_fLength;  			//subtract off the seg length
			m_vsegListItr++;  						 	// & move to next segment
			CurrSeg++;  						 		// & move to next segment
			bNewSegment=true;
			if (m_vsegListItr == m_vsegList.end())
			{
				enReturn = enLastPointOfPath;
				break;
			}
		}
		else
		{
			SegPath		+= CurrVel;
			CurrPath	+= CurrVel;
			GetPathPoint(bNewSegment, SegPath, vPath);
		}
		
	}

	return (int)enReturn;
};


int Path::GetPathPoint(bool & bNewSegment, double SegPathLength, vector<PathPoint> & RtnPt  )
{

	vPathSegment PathSeg;
	PathPoint				rPt;

	if (m_vsegListItr == m_vsegList.end())   //return the final endpoint
	{
		m_vsegListItr--;
		PathSeg=*m_vsegListItr;
		rPt.X = (long int)( (PathSeg.m_End[0] + m_XOffset) * m_UnitsToCountsX );
		rPt.Y = (long int)( (PathSeg.m_End[1] + m_YOffset) * m_UnitsToCountsY );
		rPt.Z = (long int)( (PathSeg.m_End[2] + m_ZOffset) * m_UnitsToCountsZ );
		m_vsegListItr = m_vsegList.end();
		RtnPt.push_back(rPt);
		return 0;
	};

	vLine		Line;
	vArc			Arc;
	vSpline	Spline;
	vector<double>			Pt;

	PathSeg=*m_vsegListItr;
	if (PathSeg.m_segType == enOrigin)
		return 0;

	if (PathSeg.m_segType == enLine)
	{
		Line = *(vLine*)*m_vsegListItr;
		Pt=GetPointOnLine( Line, SegPathLength );
	}
	else if (PathSeg.m_segType == enArc)
	{

		Arc = *(vArc*)*m_vsegListItr;
		if (bNewSegment ==  true)
		{
			GetArcFrameP( &Arc, &m_vCurrArcFrame1 );
			bNewSegment = false;
		}

		Pt=GetPointOnArc( Arc, SegPathLength );
	}
	else if (PathSeg.m_segType == enSpline)
	{

		Spline = *(vSpline*)*m_vsegListItr;
		Pt=GetPointOnSpline( Spline, SegPathLength );
	}
	
	int j =0;
	for (int i=0 ; i <= Pt.size()/3-1; i++)
	{
		rPt.X = (long int)( (Pt[j++] + m_XOffset) * m_UnitsToCountsX );
		rPt.Y = (long int)( (Pt[j++] + m_YOffset) * m_UnitsToCountsY );
		rPt.Z = (long int)( (Pt[j++] + m_ZOffset) * m_UnitsToCountsZ );
		RtnPt.push_back(rPt);
	}
	return 0;
}
//---------------------------------------------------------------------------
//Adds points to path buffer - should be called at regular intervals which
//are shorter than the buffer time (m_BufSize/m_PathFreq).
//
//Returns: -1 if path download is done
//         m_CurrSeg if in middle  of the path
//		   -2 if communication error

int   Path::AddPathPointsV(vector<PathPoint> & vPath)
{
	int  iServoNPoints		=0;
	int  iPointsToAdd		=0;
	long PathX[10], PathY[10], PathZ[10];
	int  iPathPoints =0;
	vector<PathPoint>::const_iterator  PathItr;
	PathPoint PPoint;

//m_CurrVel = m_CmdMaxVel = 0.0 defines a feedhold condition
//Setting m_CmdMaxVel to a non-zero value will resume path execution
//	if (m_CmdMaxVel==0.0 && m_CurrVel == 0.0) return(m_CurrSeg);
	if ( m_CmdMaxVel==0.0 ) return(-1);
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
			
			if (!ServoAddPathpoints(m_x, iPointsToAdd, PathX, m_PathFreq)) return(-2);
			if (!ServoAddPathpoints(m_y, iPointsToAdd, PathY, m_PathFreq)) return(-2);
			if (m_z)
			if (!ServoAddPathpoints(m_z, iPointsToAdd, PathZ, m_PathFreq)) return(-2);

			if (!NmcNoOp(m_x)) return(-2);                     //read num points from X
			iServoNPoints=ServoGetNPoints(m_x);

		} while ((iServoNPoints < m_BufSize) && (iServoNPoints < 87) &&
			      iPathPoints < vPath.size());
		
		if (!NmcNoOp(m_y)) 
			return(-2);  //make sure data is updated even if points are not added
			
		if (!NmcNoOp(m_z)) 
			return(-2);
			
		if (!(ServoGetAux(m_x) & PATH_MODE))    //start path mode when buffer full
		if (!ServoStartPathMode(m_group, m_leader)) return(-2);
		
	}

	return iPathPoints;
}



int   Path::CreatePathPointsP(vector<PathPoint> & vPath, vector<double> dPathLength)
{
	int iRet;

	if (dPathLength.size() == 0)
		return -1;

	vPathSegment PathSeg;
	m_vsegListItr = m_vsegList.begin();

	PathSeg=*m_vsegListItr;
	if (PathSeg.m_segType != enOrigin)
		return enNoOrigin;

	m_vsegListItr++;
	
	for (int i=0; i <= dPathLength.size()-1; i++) 
		iRet=GetPathPointsA( vPath, dPathLength[i] );

	SavePathToFile("path.txt", vPath);
	return(vPath.size());

};

int   Path::ReadPathSegment()
{
	
	list<vPathSegment*>::iterator  segListItr;

	int i;
	vLine LineSeg;
	vArc ArcSeg;
	vPathSegment *Seg =new vPathSegment;
	segListItr=m_vsegList.begin();
	for (i=0; i <= m_vsegList.size()-1; i++)
	{
		Seg=*segListItr;
		if (Seg->m_segType == enOrigin)
		{
			segListItr++;
			continue;
		}

		if (Seg->m_segType == enLine)
		{
			LineSeg=*(vLine*)*segListItr;
			segListItr++;
			continue;
		}
	
		if (Seg->m_segType == enArc)
		{
//			ArcSeg=*(ArcPath::Arc*)*segListItr;
			ArcSeg=*(vArc*)Seg;
			segListItr++;
			continue;
		}

		
	};


	return 0;
};


int Path::SavePathToFile(char *lpszFileName, vector<PathPoint> & vPath )
{


   FILE *stream2;
   vector<PathPoint>::const_iterator  PathItr;
  

   if( (stream2 = fopen( lpszFileName, "w+" )) == NULL )
   {
      printf( "The file 'data2' was not opened\n" );
	  return -1;
	  }
   

   for (int iCount=0;iCount <= vPath.size()-1; iCount++)
   {
	   fprintf( stream2, "%d %d\n", vPath[iCount].X, vPath[iCount].Y );
   }

   fclose(stream2);
   
   return 0;
}
int Path::SavePathToFile(vector<PathPoint> & vPath )
{

	if (!m_fpoints.is_open())  
		return -1;
	
	m_fpoints << "BEGIN" << endl;
	for (int iCount=0;iCount <= vPath.size()-1; iCount++)
	{
	   m_fpoints << vPath[iCount].X << " " << vPath[iCount].Y << endl;
	}
	m_fpoints << "END" << endl;

   
	return 0;
}


int Path::ClearList()
{
	
	int numberOfelements = m_vsegList.size();
	list<vPathSegment*>::iterator  vsegListItr;

	for (vsegListItr=m_vsegList.begin(); vsegListItr != m_vsegList.end(); vsegListItr++)
	{
		vPathSegment* vsegList= *vsegListItr;
		delete vsegList;
	}
	return 0;
};