#ifndef __ENUMTYPES_H__
#define __ENUMTYPES_H__

enum enIOError {enInvalidCommand=-1, enInvalidPath=-2, enFileNotOpen=-3};
enum enCommand {enStartPath=100, enEndPath, enExecutePath, enStageMove};
enum enSplineType {enBezier, enCubic, enCatmullRom};
enum enSplineState {enInitialized=0, enLengthNotSet=-1, enInvalidNumberOfLengths=-2, 
                    enExceededNumberOfSegments=-3, enEndOfCurrentSegment=-4, enLastPoint=-5, 
					enExceededNumberOfControlPts=-6, enNotInitialized=-7};

#endif
