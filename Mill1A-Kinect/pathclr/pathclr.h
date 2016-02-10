// pathclr.h

#pragma once
#include "Path.h"

using namespace System;
using namespace System::Runtime::InteropServices;
#pragma make_public(Path);
namespace pathclr {

	public ref class CPath
	{
		public:
			virtual ~CPath();
			CPath();
			int		SetPathParams(int freq, int nbuf,
						           double xscale, double yscale, double zscale,
						           double accel );
			void	SetFeedrate(double fr);
			void	SetOrigin(double xoffset, double yoffset, double zoffset);
			void	SetTangentTolerance(float theta);
			Path* GetNativePtr();
		// TODO: Add your methods for this class here.
		private:
			Path* _pPath;
	};
}
