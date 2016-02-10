using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using stageclr;

namespace XBoxJoyStick
{
    class StageControl
    {
        
        private CStage myStage;
        public CStage Stage
        {
            get
            {
                return myStage;
            }
            set
            {
                myStage = value;
            }
        }
        double _xspeed, _yspeed, _zspeed;
        double _xaccel, _yaccel, _zaccel;
        
       public void Initialize()
        {
            myStage = new CStage("COM4:");
            myStage.Rotate(180.0);

            SetMotionGains();
            SetSpeedAndAcceleration();
            InitializeCoordMotion();
        }
        public int SetScaling_3Axis()
        {
            myStage.SetEncoderCounts(0, 200.00);
            myStage.SetEncoderCounts(1, 200.00);
            myStage.SetEncoderCounts(2, 200.00);

            myStage.SetLeadScrewPitch(0, 10.00);
            myStage.SetLeadScrewPitch(1, 10.00);
            myStage.SetLeadScrewPitch(2, 20.00);

            myStage.SetMotorGearRatio(0, 5.9);
            myStage.SetMotorGearRatio(1, 5.9);
            myStage.SetMotorGearRatio(2, 5.9);

            myStage.SetPulleyRatio(0, 2.13 / 1.504);
            myStage.SetPulleyRatio(1, 2.13 / 1.504);
            myStage.SetPulleyRatio(2, 2.13 / 1.504);

            return 0;
        }
        public int SetPIDGain_3Axis()
        {
            int iProportional;
            int iDifferential;
            int iIntegral;

            int bXAxisGain = 0;
            int bYAxisGain = 0;
            int bZAxisGain = 0;

            myStage.SetGainLimits(0, 0.0, 255.0, 0.0, 2000.0, 1.0, 0.0);
            myStage.SetGainLimits(1, 0.0, 255.0, 0.0, 2000.0, 1.0, 0.0);
            myStage.SetGainLimits(2, 0.0, 255.0, 0.0, 2000.0, 1.0, 0.0);

            bXAxisGain = myStage.SetGain(0, 2144, 7680, 512);	   //DC = 0
            if (bXAxisGain == 1)
                return -1;

            bYAxisGain = myStage.SetGain(1, 1656, 4064, 128);     //DC = 0;
            if (bYAxisGain == 1)
                return -2;

            bZAxisGain = myStage.SetGain(2, 1872, 12288, 512);      //DC = 0
            if (bZAxisGain == 1)
                return -3;

            return 0;

        }
        public void InitializeCoordMotion()
        {
            myStage.SetGroupAddress(128, 0);
            myStage.StopResetMotors();
            myStage.SetPathStatus();

        }
        private void SetMotionGains()
        {
            SetScaling_3Axis();
            SetPIDGain_3Axis();
        }

        private void SetSpeedAndAcceleration()
        {
            myStage.SetVel(2.0, 2.0, 2.0);
            myStage.SetAccel(6.00, 6.00, 6.00);
            myStage.EnableAmp();
            _xspeed = 1.5; _yspeed = 1.5; _zspeed = 1.5;
            _xaccel = 6.0; _yaccel = 6.0; _zaccel = 6.0;

        }
    }
}
