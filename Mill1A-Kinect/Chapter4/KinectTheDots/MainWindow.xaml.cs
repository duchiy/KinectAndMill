/*
 * 
 *  Copyright (c) 2012 Jarrett Webb & James Ashley
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 *  documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
 *  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 *  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 *  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 *  IN THE SOFTWARE.
 * 
 * 
 */


using System;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

using Microsoft.Kinect;
using Nui=Microsoft.Kinect;
using stageclr;
using XBoxJoyStick;

namespace BeginningKinect.Chapter4.KinectTheDots
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Member Variables
        private KinectSensor _KinectDevice;
        private DotPuzzle _Puzzle;
        private int _PuzzleDotIndex;
        private Skeleton[] _FrameSkeletons;
        CStage _myStage;
        StageControl myStageControl = new StageControl();
        bool _trackRightHand = false;

        public delegate void MoveStageEx(double X, double Y, double Z);
        public event  MoveStageEx _MoveStage;
        protected void OnMoveStage(double X, double Y, double Z)
        {
            if (_MoveStage != null)
            {
                _MoveStage(X, Y, Z);
            }
        }
        
        public delegate void StopStageEx();
        public event StopStageEx _StopStage;
        protected void OnStopStage()
        {
            if (_StopStage != null)
            {
                _StopStage();
            }
        }


        #endregion Member Variables


        #region Constructor
        public MainWindow()
        {
            InitializeComponent();
            myStageControl.Initialize();
            _myStage = myStageControl.Stage;
            this._MoveStage += new MoveStageEx(MoveStage);
            this._StopStage += new StopStageEx(StopStage);
            this._Puzzle = new DotPuzzle();
            this._Puzzle.Dots.Add(new Point(200, 300));
 //           this._Puzzle.Dots.Add(new Point(1600, 300));
            //this._Puzzle.Dots.Add(new Point(1650, 400));
            //this._Puzzle.Dots.Add(new Point(1600, 500));
            //this._Puzzle.Dots.Add(new Point(1000, 500));
            //this._Puzzle.Dots.Add(new Point(1000, 600));
            //this._Puzzle.Dots.Add(new Point(1200, 700));
            //this._Puzzle.Dots.Add(new Point(1150, 800));
            //this._Puzzle.Dots.Add(new Point(750, 800));
            //this._Puzzle.Dots.Add(new Point(700, 700));
            //this._Puzzle.Dots.Add(new Point(900, 600));
            //this._Puzzle.Dots.Add(new Point(900, 500));
            //this._Puzzle.Dots.Add(new Point(200, 500));
            //this._Puzzle.Dots.Add(new Point(150, 400));

            this._PuzzleDotIndex = -1;

            this.Loaded += (s,e) =>
            {
                KinectSensor.KinectSensors.StatusChanged += KinectSensors_StatusChanged;
                this.KinectDevice = KinectSensor.KinectSensors.FirstOrDefault(x => x.Status == KinectStatus.Connected);

                DrawPuzzle(this._Puzzle);
            };
        }
        #endregion Constructor


        #region Methods
        private void KinectSensors_StatusChanged(object sender, StatusChangedEventArgs e)
        {
            switch (e.Status)
            {
                case KinectStatus.Initializing:
                case KinectStatus.Connected:
                case KinectStatus.NotPowered:
                case KinectStatus.NotReady:
                case KinectStatus.DeviceNotGenuine:
                    this.KinectDevice = e.Sensor;                                        
                    break;
                case KinectStatus.Disconnected:
                    //TODO: Give the user feedback to plug-in a Kinect device.                    
                    this.KinectDevice = null;
                    break;
                default:
                    //TODO: Show an error state
                    break;
            }
        }

        // Listing 4-5
        private void KinectDevice_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {      
            using(SkeletonFrame frame = e.OpenSkeletonFrame())
            {
                if(frame != null)
                {
                    frame.CopySkeletonDataTo(this._FrameSkeletons);
                    Skeleton skeleton = GetPrimarySkeleton(this._FrameSkeletons);
                    
                    if(skeleton == null)
                    {
                        HandCursorElement.Visibility = Visibility.Collapsed;
                    }
                    else
                    {                
                        Joint primaryHand = GetPrimaryHand(skeleton);
                        TrackHand(primaryHand);
                        MoveStage(primaryHand.Position, 0);
//                        StopStage(primaryHand.Position, 1);
//                        TrackPuzzle(primaryHand.Position);                
                    }
                }                
            }
        }


        // Listing 4-5
        private static Skeleton GetPrimarySkeleton(Skeleton[] skeletons)
        {
            Skeleton skeleton = null;

            if(skeletons != null)
            {
                //Find the closest skeleton       
                for(int i = 0; i < skeletons.Length; i++)
                {
                    if(skeletons[i].TrackingState == SkeletonTrackingState.Tracked)
                    {
                        if(skeleton == null)
                        {
                            skeleton = skeletons[i];
                        }   
                        else
                        {
                            if(skeleton.Position.Z > skeletons[i].Position.Z)
                            {
                                skeleton = skeletons[i];
                            }
                        }
                    }
                }
            }

            return skeleton;
        }
        
        
        // Listing 4-6                       
        private static Joint GetPrimaryHand(Skeleton skeleton)
        {
            Joint primaryHand = new Joint();

            if(skeleton != null)
            {
                primaryHand     = skeleton.Joints[JointType.HandLeft];
                Joint righHand  = skeleton.Joints[JointType.HandRight];


                if(righHand.TrackingState != JointTrackingState.NotTracked)
                {
                    if(primaryHand.TrackingState == JointTrackingState.NotTracked)
                    {
                        primaryHand = righHand;
                    }
                    else
                    {
                        if(primaryHand.Position.Z > righHand.Position.Z)
                        {
                            primaryHand = righHand;
                        }                    
                    }    
                }
            }

            return primaryHand;
        }

      
        // Listing 4-7
        private void TrackHand(Joint hand)
        {
            if(hand.TrackingState == JointTrackingState.NotTracked)
            {
                HandCursorElement.Visibility = Visibility.Collapsed;
            }
            else
            {
                HandCursorElement.Visibility = Visibility.Visible;


                DepthImagePoint point = this._KinectDevice.CoordinateMapper.MapSkeletonPointToDepthPoint(hand.Position, this._KinectDevice.DepthStream.Format);
                point.X = (int) ((point.X * LayoutRoot.ActualWidth/_KinectDevice.DepthStream.FrameWidth) - (HandCursorElement.ActualWidth / 2.0));
                point.Y = (int)((point.Y * LayoutRoot.ActualHeight / _KinectDevice.DepthStream.FrameHeight) - (HandCursorElement.ActualHeight / 2.0));

                Canvas.SetLeft(HandCursorElement, point.X);
                Canvas.SetTop(HandCursorElement, point.Y);                

                if(hand.JointType == JointType.HandRight)
                {
                    HandCursorScale.ScaleX = 1;                    
                }
                else
                {
                    HandCursorScale.ScaleX = -1;                    
                }
            }
        }

        
        // Listing 4-10 
        private void DrawPuzzle(DotPuzzle puzzle)
        {
            PuzzleBoardElement.Children.Clear();

            if(puzzle != null)
            {
                Grid dotContainer   = new Grid();                    
                dotContainer.Width  = 175;
                dotContainer.Height = 175;
                dotContainer.Children.Add(new Ellipse { Fill = Brushes.Green });

                TextBlock dotLabel              = new TextBlock();                    
                dotLabel.Text                   = "START";
                dotLabel.Foreground             = Brushes.White;
                dotLabel.FontSize               = 40;
                dotLabel.HorizontalAlignment    = HorizontalAlignment.Center;
                dotLabel.VerticalAlignment      = VerticalAlignment.Center;
                dotContainer.Children.Add(dotLabel);

                //Position the UI element centered on the dot point
                Canvas.SetTop(dotContainer, puzzle.Dots[0].Y - (dotContainer.Height / 2) );
                Canvas.SetLeft(dotContainer, puzzle.Dots[0].X - (dotContainer.Width / 2));
                PuzzleBoardElement.Children.Add(dotContainer);

                //Grid dotContainerStop = new Grid();
                //dotContainerStop.Width = 175;
                //dotContainerStop.Height = 175;
                //dotContainerStop.Children.Add(new Ellipse { Fill = Brushes.Red });

                //TextBlock dotLabelStop = new TextBlock();
                //dotLabelStop.Text = "STOP";
                //dotLabelStop.Foreground = Brushes.White;
                //dotLabelStop.FontSize = 40;
                //dotLabelStop.HorizontalAlignment = HorizontalAlignment.Center;
                //dotLabelStop.VerticalAlignment = VerticalAlignment.Center;
                //dotContainerStop.Children.Add(dotLabelStop);

                ////Position the UI element centered on the dot point
                //Canvas.SetTop(dotContainerStop, puzzle.Dots[1].Y - (dotContainer.Height / 2));
                //Canvas.SetLeft(dotContainerStop, puzzle.Dots[1].X - (dotContainer.Width / 2));
                //PuzzleBoardElement.Children.Add(dotContainerStop);                    

            }
        }
         

        // Listing 4-11
        private void TrackPuzzle(SkeletonPoint position)
        {                
            if(this._PuzzleDotIndex == this._Puzzle.Dots.Count)
            {
                //Do nothing - Game is over
            }
            else
            {            
                Point dot;
                        
                if(this._PuzzleDotIndex + 1 < this._Puzzle.Dots.Count)
                {
                    dot = this._Puzzle.Dots[this._PuzzleDotIndex + 1];            
                }
                else
                {
                    dot = this._Puzzle.Dots[0];
                }


                DepthImagePoint point = this._KinectDevice.CoordinateMapper.MapSkeletonPointToDepthPoint(position, _KinectDevice.DepthStream.Format);
                point.X = (int) (point.X * LayoutRoot.ActualWidth/_KinectDevice.DepthStream.FrameWidth);
                point.Y = (int)(point.Y * LayoutRoot.ActualHeight / _KinectDevice.DepthStream.FrameHeight);
                Point handPoint = new Point(point.X, point.Y);  


                Point dotDiff = new Point(dot.X - handPoint.X, dot.Y - handPoint.Y);
                double length = Math.Sqrt(dotDiff.X * dotDiff.X + dotDiff.Y * dotDiff.Y);

                int lastPoint = this.CrayonElement.Points.Count - 1;

                if(length < 25)
                {
                    //Cursor is within the hit zone

                    if(lastPoint > 0)
                    {
                        //Remove the working end point
                        this.CrayonElement.Points.RemoveAt(lastPoint);
                    }

                    //Set line end point
                    this.CrayonElement.Points.Add(new Point(dot.X, dot.Y));

                    //Set new line start point
                    this.CrayonElement.Points.Add(new Point(dot.X, dot.Y));

                    //Move to the next dot 
                    this._PuzzleDotIndex++;

                    if(this._PuzzleDotIndex == this._Puzzle.Dots.Count)
                    {
                        //Notify the user that the game is over
                    }
                }
                else
                {
                    if(lastPoint > 0)
                    {   
                        //To refresh the Polyline visual you must remove the last point, update and add it back                     
                        Point lineEndpoint = this.CrayonElement.Points[lastPoint];
                        this.CrayonElement.Points.RemoveAt(lastPoint);
                        lineEndpoint.X = handPoint.X;
                        lineEndpoint.Y = handPoint.Y;
                        this.CrayonElement.Points.Add(lineEndpoint);
                    }
                }
            }
        }
        public void MoveStage(double deltaX, double deltaY, double deltaZ)
        {
            double X = 0.0, Y = 0.0, Z = 0.0;
            _myStage.GetPos(out X, out Y, out Z);
            _myStage.MoveTo(X + deltaX, Y+deltaY, 0.0, false);

        }
        public void StopStage()
        {
            _myStage.Stop();

        }

        private void MoveStage(SkeletonPoint position, int pointNumber)
        {


            DepthImagePoint point = this._KinectDevice.CoordinateMapper.MapSkeletonPointToDepthPoint(position, _KinectDevice.DepthStream.Format);
            point.X = (int)(point.X * LayoutRoot.ActualWidth / _KinectDevice.DepthStream.FrameWidth);
            point.Y = (int)(point.Y * LayoutRoot.ActualHeight / _KinectDevice.DepthStream.FrameHeight);
            Point handPoint = new Point(point.X, point.Y);
            double delta = 0.05;
            double deltaX = 0.0;
            double deltaY = 0.0;
            Point dot;
            dot = this._Puzzle.Dots[pointNumber];
            Point dotDiff = new Point(dot.X - handPoint.X, dot.Y - handPoint.Y);
            double length = Math.Sqrt(dotDiff.X * dotDiff.X + dotDiff.Y * dotDiff.Y);

            if ((length > 50) && (length < 150))
            {

                deltaX = delta * dotDiff.X / length;
                deltaY = delta * dotDiff.Y / length;

                MoveStage(deltaX, deltaY, 0.0);

            }
            else
            {

                StopStage();
            }

        }
        private void StopStage(SkeletonPoint position, int pointNumber)
        {


            DepthImagePoint point = this._KinectDevice.CoordinateMapper.MapSkeletonPointToDepthPoint(position, _KinectDevice.DepthStream.Format);
            point.X = (int)(point.X * LayoutRoot.ActualWidth / _KinectDevice.DepthStream.FrameWidth);
            point.Y = (int)(point.Y * LayoutRoot.ActualHeight / _KinectDevice.DepthStream.FrameHeight);
            Point handPoint = new Point(point.X, point.Y);
            Point dot;
            dot = this._Puzzle.Dots[pointNumber];
            Point dotDiff = new Point(dot.X - handPoint.X, dot.Y - handPoint.Y);
            double length = Math.Sqrt(dotDiff.X * dotDiff.X + dotDiff.Y * dotDiff.Y);

            if (length < 100)
            {

                StopStage();

            }

        }
        #endregion Methods


        #region Properties
        public KinectSensor KinectDevice 
        {
            get { return this._KinectDevice; }
            set
            {
                if(this._KinectDevice != value)
                {
                    //Uninitialize
                    if(this._KinectDevice != null)
                    {
                        this._KinectDevice.Stop();
                        this._KinectDevice.SkeletonFrameReady -= KinectDevice_SkeletonFrameReady;
                        this._KinectDevice.SkeletonStream.Disable();
                        SkeletonViewerElement.KinectDevice = null;
                        this._FrameSkeletons = null;
                    }
                   
                    this._KinectDevice = value;

                    //Initialize
                    if(this._KinectDevice != null)
                    {
                        if(this._KinectDevice.Status == KinectStatus.Connected)
                        {
                            this._KinectDevice.SkeletonStream.Enable();
                            this._FrameSkeletons = new Skeleton[this._KinectDevice.SkeletonStream.FrameSkeletonArrayLength];                        
                            this._KinectDevice.Start(); 

                            SkeletonViewerElement.KinectDevice = this.KinectDevice;
                            this.KinectDevice.SkeletonFrameReady += KinectDevice_SkeletonFrameReady;                            
                        }
                    }                
                }
            }
        }        
        #endregion Properties
    }
}

