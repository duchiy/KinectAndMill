// This is the main DLL file.

#include "stdafx.h"

#include "pathclr.h"
#include <vcclr.h>

using namespace pathclr;
CPath::CPath()
{
	_pPath = Singleton<Path>::InstancePtr();
		
};
CPath::~CPath()
{
   if (this->_pPath != 0)
   {
      delete this->_pPath;
      this->_pPath = 0;
   }
};
int	CPath::SetPathParams(int freq, int nbuf,
						 double xscale, double yscale, double zscale,
						 double accel )
{
	_pPath = Singleton<Path>::InstancePtr();
	_pPath->SetPathParams(freq, nbuf,
						 xscale, yscale, zscale,
						 accel );
	return 0;
};
void CPath::SetFeedrate(double fr)
{

	_pPath->SetFeedrate(fr);
};
void CPath::SetOrigin(double xoffset, double yoffset, double zoffset)
{

	_pPath->SetOrigin(xoffset, yoffset, zoffset);
};
void CPath::SetTangentTolerance(float theta)
{
	_pPath->SetTangentTolerance(theta);

};
Path* CPath::GetNativePtr()
{
	 return _pPath;
};