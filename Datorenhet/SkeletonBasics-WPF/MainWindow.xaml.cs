//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System;
    using System.Windows.Input;
    using System.Windows.Controls;
    using System.IO;
    using System.IO.Ports;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Forms;
    using Microsoft.Kinect;
    //using MLApp;
    using System.Windows.Threading;
    // using SlimDX.DirectInput;
    using System.Text.RegularExpressions; //For use of Regex.Replace
    using System.Linq;
    using System.Collections.Generic;
    using System.Diagnostics;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //private MLApp matlab = new MLApp();
        private DispatcherTimer out_timer = new DispatcherTimer();

        private Stopwatch stopwatch = new Stopwatch();

        private double[] old_positions = { 5, 0, -0.1061, 0.1061, -0.2058, 0.2058, 5, 0};
        private double[] new_positions = { 0, 0, -0.1061, 0.1061, -0.2058, 0.2058, 0, 0};

        private Queue<double> x_queue = new Queue<double>();
        private Queue<double> y_queue = new Queue<double>();
        private Queue<double> x_queue_fix = new Queue<double>();
        private Queue<double> y_queue_fix = new Queue<double>();
        private Queue<double> theta_queue = new Queue<double>();

        //buffer for angles to send
        private short[] angle_buffer = new short[4];

        // Width of output drawing
        private const float RenderWidth = 640.0f;

        // Height of our output drawing
        private const float RenderHeight = 480.0f;

        // Thickness of drawn joint lines
        private const double JointThickness = 6;

        // Thickness of body center ellipse
        private const double BodyCenterThickness = 10;

        // Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;

        // Brush used to draw skeleton center point
        private readonly Brush centerPointBrush = Brushes.Blue;

        // Brush used for drawing joints that are currently tracked
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        // Brush used for drawing joints that are currently inferred       
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        // Pen used for drawing bones that are currently tracked
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        // Pen used for drawing bones that are currently inferred
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        // Active Kinect sensor
        private KinectSensor sensor;

        // Drawing group for skeleton rendering output
        private DrawingGroup drawingGroup;

        // Drawing image that we will display
        private DrawingImage imageSource;

        // Bluetooth
        private bool connected = false;

        // Baud rate for COM-port
        private int baudRate = 115200;

        // Serialport for sending data
        private SerialPort inPort = new SerialPort();

        // Bool för att skicka koordinater
        private bool send = false;

        // Armläge
        private bool arm = false;

        // Kalibrera för golv och tejp
        private bool floor = true;

        // Om roboten ska gå in i arm-mode när den kommer fram
        private bool mission = false;

        // Tar emot vinklar
        private bool receive_angle = false;

        // Håller koll på vilket servo vinklarna kommer ifrån
        private int counter = 1;


        // Coordinates for joints (shoulder is set to be origo)
        //RIGHT ARM
        private float[] rightHand = new float[3];
        private float[] rightWrist = new float[3];
        private float[] rightElbow = new float[3];
        private float[] rightShoulder = new float[3];
        // LEFT ARM
        private float[] leftHand = new float[3];
        private float[] leftWrist = new float[3];
        private float[] leftElbow = new float[3];
        private float[] leftShoulder = new float[3];

        private double[] angles = new double[4];
        double av_x;
        double av_y;
        double av_theta;

        /*double last_sent_rotation;
        int ignored_values = 0;
        int sent_values = 0;*/

        //Only update coordinates 32/3 per second
        private int[] cUpdateIterator = new int[4] { 3, 3, 3, 3 };

        //double[] pos = { 0, 0, -0.1061, 0.1061, -0.2058, 0.2058, -0.3154, 0.3154 };
        private double[] pos = { 0, 0, -0.1061, 0.1061, -0.2058, 0.2058 };
        private string buffer = "";
        private string currentInfo = "";

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// 
        public MainWindow()
        {
            InitializeComponent();
            out_timer.Interval = new TimeSpan(0, 0, 0, 0, 8);
            out_timer.Tick += new EventHandler(out_timer_Tick);
            //matlab.Execute(@"cd C:\Users\Jacob\Desktop\Projekt\SkeletonBasics-WPF");
        }

        private void out_timer_Tick(object sender, EventArgs e)
        {

             double delta_x = new_positions[6] - old_positions[6];
            double delta_y = new_positions[7] - old_positions[7];
             double diff_xy = Math.Sqrt(delta_x * delta_x + delta_y * delta_y);
           /* if (Plot_checkBox.IsChecked.Value)
            {
                try
                {
                    object result = null;
                    //matlab.Feval("diag_x", 0, out result, x_queue.ToArray(),y_queue.ToArray());

                }
                catch (Exception ex)
            {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();
                    textBox_Terminal.AppendText("[" + dtn + "] Matlab: " + ex.Message + "\n");
            }
        }*/


            if (arm && send)
            {
                CalculateAngles(av_x,av_y,av_theta);
                if (new_positions[4] != -0.2058)
                    SendAngles((short)angles[0], (short)angles[1], (short)angles[2], (short)angles[3]);
                //last_sent_rotation = angles[0];
                //Array.Copy(new_positions, old_positions, 8);
            }
        }

        /*public Joystick[] getSticks()
        {
            List<SlimDX.DirectInput.Joystick> temp = new List<SlimDX.DirectInput.Joystick>();
            foreach(DeviceInstance device in Input.GetDevices(DeviceClass.GameController, DeviceEnumerationFlags.AttachedOnly))
            {
                try
                {
                    stick = new SlimDX.DirectInput.Joystick(Input, device.InstanceGuid);
                    stick.Acquire();

                    foreach(DeviceObjectInstance deviceObject in stick.GetObjects())
                    {
                        if((deviceObject.ObjectType & ObjectDeviceType.Axis) != 0)
                        {
                            stick.GetObjectPropertiesById((int)deviceObject.ObjectType).SetRange(-100, 100);
                        }
                    }
                    temp.Add(stick);
                }
                catch(DirectInputException) { }
            }
            return temp.ToArray();
        }

        private void stickHandle(Joystick stick, int id)
        {
            JoystickState state = new JoystickState();
            state = stick.GetCurrentState();

            xvalue = state.X;
            yvalue = state.Y;
            zvalue = state.Z;
            MouseMove(xvalue, yvalue);
            bool[] buttons = state.GetButtons();

            if(id == 0)
            {
                if(buttons[0])
                {
                    //....
                }
            }
        }

        public void MouseMove (int posx, int posy)
        {
            Cursor.Position = new Point(Cursor.Position.X + posx, Cursor.Position.Y + posY);
            Cursor.Clip = new Rectangle(this.Location, this.Size);
        }

        private void timer_Tick(object sender, EventArgs e)
        {
            for (int i=0; i<Sticks.Length; i++)
            {
                stickHandle(Sticks[i], i);
            }
        }*/

        private void printCoordinates()
        {
            stopwatch.Start();
            string[] temp = new string[3] { " X: ", " Y: ", " Z: " };
            rightHandStatus.Text = "Right Hand: ";
            rightWristStatus.Text = "Right Wrist: ";
            rightElbowStatus.Text = "Right Elbow: ";
            rightShoulderStatus.Text = "Right Shoulder: ";
            rh_ref_rs_Status.Text = "Hand-ref-Shoulder: ";


            for (int i = 0; i <= 2; i++)
            {
                rightHandStatus.Text += Environment.NewLine + temp[i] + Math.Round(rightHand[i], 3);
                rightWristStatus.Text += Environment.NewLine + temp[i] + Math.Round(rightWrist[i], 3);
                rightElbowStatus.Text += Environment.NewLine + temp[i] + Math.Round(rightElbow[i], 3);
                rightShoulderStatus.Text += Environment.NewLine + temp[i] + Math.Round(rightShoulder[i], 3);
                rh_ref_rs_Status.Text += Environment.NewLine + temp[i] + Math.Round(rightHand[i] - rightShoulder[i], 3);
            }

            /*
            if (!checkBox_Mode.IsChecked.Value) //För att mata in koordinater in i textboxarna
            {*/

            double x_pos = rightHand[0] - rightShoulder[0];
            double y_pos = rightHand[1] - rightShoulder[1];
            double z_pos = rightHand[2] - rightShoulder[2];
            double theta;

            if (x_queue != null && x_queue.Count() == 10)
            {
                double[] x_array = x_queue.ToArray();
                double[] y_array = y_queue.ToArray();
                double[] theta_array = theta_queue.ToArray();
                av_x = x_array.AsEnumerable().Average();
                av_y = y_array.AsEnumerable().Average();
                if(!theta_lock_checkBox.IsChecked.Value)
                av_theta = theta_array.AsEnumerable().Average();

                X_BOX.Text = "" + Math.Round(av_x, 3);
                Y_BOX.Text = "" + Math.Round(av_y, 3);
                theta_BOX.Text = "" + Math.Round(av_theta, 3);

                x_queue.Dequeue();
                y_queue.Dequeue();
                theta_queue.Dequeue();
  
            }

            x_queue.Enqueue(Math.Round(Math.Sqrt(x_pos * x_pos + z_pos * z_pos), 3) * 4 / 5);
            y_queue.Enqueue(Math.Round(y_pos, 3) * 4 / 5);
            if (z_pos <= 0)
            {
                theta = Math.Asin(x_pos / (Math.Sqrt(x_pos * x_pos + z_pos * z_pos)));
            }
            else
            {
                theta = Math.PI - Math.Asin(x_pos / (Math.Sqrt(x_pos * x_pos + z_pos * z_pos)));
            }
            theta = theta * (180.0 / Math.PI);
            if (Math.Abs(theta) > 45)
            {
                theta = Math.Sign(theta) * 45;
            }
            theta_queue.Enqueue(theta);
            stopwatch.Stop();
            // Angle4.Text = "" + System.Math.Round(rightHand[2] - rightShoulder[2], 3);
            /*}
        */
        }

        private void GetCoordinates(Skeleton skeleton)
        {
            // RIGHT ARM
            Joint rh = skeleton.Joints[JointType.HandRight]; // Right hand
            Joint rw = skeleton.Joints[JointType.WristRight]; // Right wrist
            Joint re = skeleton.Joints[JointType.ElbowRight]; // Right elbow
            Joint rs = skeleton.Joints[JointType.ShoulderRight]; // Right shoulder

            // LEFT ARM
            Joint lh = skeleton.Joints[JointType.HandLeft];
            Joint lw = skeleton.Joints[JointType.WristLeft];
            Joint le = skeleton.Joints[JointType.ElbowLeft];
            Joint ls = skeleton.Joints[JointType.ShoulderLeft];


            rightHand[0] = rh.Position.X;
            rightHand[1] = rh.Position.Y;
            rightHand[2] = rh.Position.Z;
            rightWrist[0] = rw.Position.X;
            rightWrist[1] = rw.Position.Y;
            rightWrist[2] = rw.Position.Z;
            rightElbow[0] = re.Position.X;
            rightElbow[1] = re.Position.Y;
            rightElbow[2] = re.Position.Z;
            rightShoulder[0] = rs.Position.X;
            rightShoulder[1] = rs.Position.Y;
            rightShoulder[2] = rs.Position.Z;

            leftHand[0] = lh.Position.X;
            leftHand[1] = lh.Position.Y;
            leftHand[2] = lh.Position.Z;
            leftWrist[0] = lw.Position.X;
            leftWrist[1] = lw.Position.Y;
            leftWrist[2] = lw.Position.Z;
            leftElbow[0] = le.Position.X;
            leftElbow[1] = le.Position.Y;
            leftElbow[2] = le.Position.Z;
            leftShoulder[0] = ls.Position.X;
            leftShoulder[1] = ls.Position.Y;
            leftShoulder[2] = ls.Position.Z;
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();

            if (leftHand[1] > skeleton.Joints[JointType.Head].Position.Y) // Send if left hand is above the head
            {
                if (!send)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Sending Kinect data\n");
                    send = true;
                    out_timer.Start();
                }

            }
            if (leftHand[0] > skeleton.Joints[JointType.Head].Position.X) // Don't Send if left hand right to the head
            {
                if (send)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Stopped sending Kinect data\n");
                    send = false;
                    out_timer.Stop();
                }
            }
            printCoordinates();
            //CalculateAngles();
            //SendCoordinates();
        }

        /// Draws indicators to show which edges are clipping skeleton data
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// Execute startup tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                    this.statusBarText.Text = "Kinect Ready";
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }


        /// Execute shutdown tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            //matlab.Quit();
            send = false;
            out_timer.Stop();
            if (null != this.sensor)
            {
                sensor.Stop();
            }
            if (inPort.IsOpen)
            {
                inPort.Close();
            }
        }


        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// Draws a skeleton's bones and joints
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);
            GetCoordinates(skeleton);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }


        /// Draws a bone line between two joints
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
            //GetCoordinates(joint1);
        }


        //private void BluetoothStatus
        private void SendAngles(short v1, short v2, short v3, short v4)
        {

            //Prova skicka: 0x01ff (0xff, 0x01), 0x0333 (0x33,0x03), 0x0333 (0x33,0x03), 0x1fff, (0xff,0x01)
            /*
            byte v1_m = (byte)(((1024 * v1 + 61200) / 300) / 256);
            byte v1_l = (byte)(((1024 * v1 + 61200) / 300) % 256);
            byte v2_m = (byte)(((1024 * v2 + 61200) / 300) / 256);
            byte v2_l = (byte)(((1024 * v2 + 61200) / 300) % 256);
            byte v3_m = (byte)(((1024 * v3 + 61200) / 300) / 256);
            byte v3_l = (byte)(((1024 * v3 + 61200) / 300) % 256);
            byte v4_m = (byte)(((1024 * v4 + 61200) / 300) / 256);
            byte v4_l = (byte)(((1024 * v4 + 61200) / 300) % 256);
            */

            byte v1_m = (byte)(v1 >> 8);
            byte v1_l = (byte)(v1 & 0xff);
            byte v2_m = (byte)(v2 >> 8);
            byte v2_l = (byte)(v2 & 0xff);
            byte v3_m = (byte)(v3 >> 8);
            byte v3_l = (byte)(v3 & 0xff);
            byte v4_m = (byte)(v4 >> 8);
            byte v4_l = (byte)(v4 & 0xff);


            byte[] data = new byte[] { v1_l, v1_m, v2_l, v2_m };
            byte[] data2 = new byte[] { v3_l, v3_m, v4_l, v4_m };

            if (inPort.IsOpen)
            {
                try
                {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();
                    char[] command = { '!' }; // Kommando som säger att vinklar kommer
                    char[] start1 = { 'A' };
                    //Skicka första halvan
                    SendMove(command);
                    SendMove(start1);
                    inPort.Write(data, 0, 4);

                    //Skicka andra halvan
                    char[] start2 = { 'B' };
                    SendMove(command);
                    SendMove(start2);
                    inPort.Write(data2, 0, 4);

                    textBox_Terminal.AppendText("[" + dtn + "] Sent: " + data + "\n");
                }
                catch (System.Exception ex)
                {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();
                    textBox_Terminal.AppendText("[" + dtn + "] Bluetooth: " + ex.Message + "\n");
                }
            }
            else
            {
                DateTime dt = DateTime.Now;
                string dtn = dt.ToShortTimeString();
                textBox_Terminal.AppendText("[" + dtn + "] Bluetooth: Port is not open\n");
            }

        }

        private double angle_map(double s, double a1, double a2, double b1, double b2)
        {
            return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
        }

        private void CalculateAngles(double x, double y, double theta_value)
        {
            /*
            //Set rightShoulder as origo
            float[] tempRH = rightHand;
            float[] tempRW = rightWrist;
            float[] tempRE = rightElbow;
            float[] tempRS = rightShoulder;

            for (int i = 0; i <= 2; i++)
            {
                tempRH[i] = tempRH[i] - tempRW[i];
                tempRW[i] = tempRW[i] - tempRE[i];
                tempRE[i] = tempRE[i] - tempRS[i];
                tempRS[i] = 0;
            }

            double radiusS2E = System.Math.Sqrt(tempRE[0] * tempRE[0] + tempRE[1] * tempRE[1]); // Radius Shoulder 2 Wrist
            double radiusE2W = System.Math.Sqrt(tempRW[0] * tempRW[0] + tempRW[1] * tempRW[1]); // Radius Elbow 2 wrist
            double radiusW2H = System.Math.Sqrt(tempRH[0] * tempRH[0] + tempRH[1] * tempRH[1]); // Radius Wrist 2 hand
            // Rotation angle - we want arctan(0)=0 
            angles[0] = System.Math.Round(System.Math.Atan(tempRE[0] / tempRE[2]) * 180 / System.Math.PI); // theta = arctan(x/z)
            if (angles[0] < 0)
            {
                angles[0] = angles[0] + 180;
            }
            angles[1] = System.Math.Round(System.Math.Acos(tempRE[1] / radiusS2E) * 180 / System.Math.PI); // v1 = arccos(y/r)
            angles[2] = System.Math.Round(System.Math.Acos(tempRW[1] / radiusE2W) * 180 / System.Math.PI); // v2 = arccos(y/r)
            angles[3] = 180 - System.Math.Round(System.Math.Acos(tempRH[1] / radiusW2H) * 180 / System.Math.PI); // v3 = arccos(y/r)
            */

            if (arm)
            {
                object result = null;
                double[] t = { x, y + 0.1 };

                //textBox_Terminal.AppendText("Sent: " + x + "," + y + "\n");
                try
                {
                    //matlab.Feval("FABRIK_2", 2, out result, pos, t);

                    object[] res = result as object[];

                    double[,] new_pos = (double[,])res[0];
                    double[,] theta = (double[,])res[1];

                    int i = 0;
                    foreach (double d in new_pos)
                    {
                        new_positions[i] = d;
                        i++;
                    }

                    i = 1;
                    foreach (double d in theta)
                    {
                        angles[i] = d;
                        i++;
                    }
                    double rotation = theta_value;
                    angles[1] = 180 - angles[1];
                    Angle1.Text = "" + Math.Round(angles[0]);
                    Angle2.Text = "" + Math.Round(angles[1]);
                    Angle3.Text = "" + Math.Round(angles[2]);
                    Angle4.Text = "" + Math.Round(angles[3]);
                    rotation = -rotation;
                    angles[0] = Math.Round(angle_map(rotation, -45, 45, 2028, 2805));
                    angles[1] = Math.Round(angle_map(angles[1], 0, 180, 1025, 3058));
                    angles[2] = Math.Round(angle_map(angles[2], -63, 72, 1365, 2891));
                    angles[3] = Math.Round(angle_map(angles[3], -90, 90, 780, 2900));
                    // textBox_Terminal.AppendText(output + "\n");

                    /*if (old_positions[6] == 5)
                        Array.Copy(new_positions, old_positions, 8);*/

                }
                catch (Exception ex)
                {
                    this.Dispatcher.Invoke((Action)(() =>
                    {
                        textBox_Terminal.AppendText("HELP: " + ex + "\n");
                    }));


                }
            }
        }


        /// <summary>
        /// Skickar ut koordinaterna för högerhanden till COM-porten.
        /// </summary>
        /*private void SendCoordinates()
        {
            if (send)
            {
                char[] command = { 'O' };
                SendMove(command);       // Kommando som säger att koordinater kommer
                SendValue(rightHand[0] - rightShoulder[0]); // Skickar x-koordinaten för högerhanden i referens till axeln.
                SendValue(rightHand[1] - rightShoulder[1]); // Skickar y-koordinaten för högerhanden i referens till axeln.
                SendValue(rightHand[2] - rightShoulder[2]); // Skickar z-koordinaten för högerhanden i referens till axeln.
            }
        }*/


        /// Handles the checking or unchecking of the seated mode combo box
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        /// Handles the checking or unchecking of the near mode combo box
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxNearModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxNearMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.DepthStream.Range = DepthRange.Near;
                }
                else
                {
                    this.sensor.DepthStream.Range = DepthRange.Default;
                }
            }
        }

        private void ConnectionButton_Click(object sender, RoutedEventArgs e)
        {

            if (!connected)
            {
                if (BlueToothConnections.Items.Count > 0)
                {
                    try
                    {
                        DateTime dt = DateTime.Now;
                        string dtn = dt.ToShortTimeString();
                        inPort.PortName = BlueToothConnections.Text;
                        inPort.BaudRate = baudRate;
                        inPort.Parity = Parity.None;
                        inPort.StopBits = StopBits.One;
                        inPort.DataBits = 8;
                        inPort.WriteTimeout = 50;
                        inPort.DataReceived += new SerialDataReceivedEventHandler(inPort_DataReceived);
                        inPort.Open();
                        textBox_Terminal.AppendText("[" + dtn + "]" + " Bluetooth: " + BlueToothConnections.Text + " is opened\n");
                        ConnectionButton.Content = "Disconnect";
                        BlueToothConnections.Items.Clear();
                        connected = true;
                    }
                    catch (System.Exception ex)
                    {
                        DateTime dt = DateTime.Now;
                        string dtn = dt.ToShortTimeString();
                        textBox_Terminal.AppendText("[" + dtn + "] " + "Bluetooth: " + ex.Message + "\n");
                    }
                }
                else
                {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();
                    textBox_Terminal.AppendText("[" + dtn + "] " + "Bluetooth: Search for connections.\n");
                }
            }
            else
            {
                DateTime dt = DateTime.Now;
                string dtn = dt.ToShortTimeString();
                if (inPort.IsOpen)
                {
                    inPort.DiscardInBuffer();
                    inPort.DiscardOutBuffer();
                    inPort.Close();
                    inPort.DataReceived -= new SerialDataReceivedEventHandler(inPort_DataReceived);
                }
                ConnectionButton.Content = "Connect";
                textBox_Terminal.AppendText("[" + dtn + "] " + "Bluetooth: Is disconnected\n");
                connected = false;
            }

        }


        private void SearchButton_Click(object sender, RoutedEventArgs e)
        {
            string[] ports = SerialPort.GetPortNames();
            if (ports.Length > 0)
            {
                BlueToothConnections.Items.Clear();
                ConnectionButton.IsEnabled = true;
                for (int i = 0; i < ports.Length; i++)
                    BlueToothConnections.Items.Add(ports[i]);
            }
            else
            {
                DateTime dt = DateTime.Now;
                string dtn = dt.ToShortTimeString();
                textBox_Terminal.AppendText("[" + dtn + "]" + " Bluetooth: No connections available\n");
            }
        }


        /// <summary>
        /// Knappen som skickar antingen vinklar eller parametrar.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Send_Click(object sender, RoutedEventArgs e)
        {
            if (arm)//!checkBox_Mode.IsChecked.Value)
            {
                short v1 = short.Parse(Angle1.Text);
                short v2 = short.Parse(Angle2.Text);
                short v3 = short.Parse(Angle3.Text);
                short v4 = short.Parse(Angle4.Text);
                SendAngles(v1, v2, v3, v4);

                /*try
                {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();
                    byte v1 = byte.Parse(Angle1.Text);
                    byte v2 = byte.Parse(Angle2.Text);
                    byte v3 = byte.Parse(Angle3.Text);
                    byte v4 = byte.Parse(Angle4.Text);
                    if (v1 >= 0 && v1 <= 180 && v2 >= 0 && v2 <= 180 && v3 >= 0 && v3 <= 180 && v4 >= 0 && v4 <= 180)
                    {
                        if (connected)
                        {
                            char[] command = { 'P' }; // Kommando som säger att vinklar kommer
                            SendMove(command);
                            SendAngles(v1, v2, v3, v4);
                            textBox_Terminal.AppendText("[" + dtn + "]" + "Send: " + "Angles sent\n");
                        }
                        else
                        {
                            textBox_Terminal.AppendText("[" + dtn + "]" + "Send: " + "No connection\n");
                        }

                    }
                    else
                    {
                        textBox_Terminal.AppendText("[" + dtn + "]" + "Send: " + "Angles must be 0-180\n");
                    }


                }
                catch (System.Exception ex)
                {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();

                    textBox_Terminal.AppendText("[" + dtn + "]" + "Send: " + ex.Message + "\n");
                }*/
            }
            else
            {
                try
                {
                    if (checkBox_1.IsChecked.Value && !arm)
                    {
                        UInt32 speed = UInt32.Parse(Angle1.Text);
                        if (speed >= 0 && speed <= 255) // Den önskade farten måste ha ett giltligt värde
                        {
                            SendStartbit();
                            byte[] command = { 1 }; // Kommando som säger att farten kommer
                            SendByte(command);
                            SendSpeed(speed);
                        }
                        else
                        {
                            DateTime dt = DateTime.Now;
                            string dtn = dt.ToShortTimeString();
                            textBox_Terminal.AppendText("[" + dtn + "]: " + "Duty cycle has to be between 0 and 255\n");
                        }
                    }
                    if (checkBox_2.IsChecked.Value && !arm)
                    {
                        float P = float.Parse(Angle2.Text);
                        if (P >= 0)
                        {
                            SendStartbit();
                            byte[] command = { 2 }; // Kommando som säger att P-parametern kommer
                            SendByte(command);
                            SendValue(P);
                        }
                        else
                        {
                            DateTime dt = DateTime.Now;
                            string dtn = dt.ToShortTimeString();
                            textBox_Terminal.AppendText("[" + dtn + "]: " + "P has to be positive\n");
                        }
                    }
                    if (checkBox_3.IsChecked.Value && !arm)
                    {
                        float I = float.Parse(Angle3.Text);
                        if (I >= 0)
                        {
                            SendStartbit();
                            byte[] command = { 3 }; // Kommando som säger att I-parametern kommer
                            SendByte(command);
                            SendValue(I);
                        }
                        else
                        {
                            DateTime dt = DateTime.Now;
                            string dtn = dt.ToShortTimeString();
                            textBox_Terminal.AppendText("[" + dtn + "]: " + "I has to be positive\n");
                        }
                    }
                    if (checkBox_4.IsChecked.Value && !arm)
                    {
                        float D = float.Parse(Angle4.Text);
                        if (D >= 0)
                        {
                            SendStartbit();
                            byte[] command = { 4 }; // Kommando som säger att D-parametern kommer
                            SendByte(command);
                            SendValue(D);
                        }
                        else
                        {
                            DateTime dt = DateTime.Now;
                            string dtn = dt.ToShortTimeString();
                            textBox_Terminal.AppendText("[" + dtn + "]: " + "D has to be positive\n");
                        }
                    }
                }
                catch (System.Exception ex)
                {
                    DateTime dt = DateTime.Now;
                    string dtn = dt.ToShortTimeString();
                    textBox_Terminal.AppendText("[" + dtn + "]" + "Send: " + ex.Message + "\n");
                }
            }
        }


        /// <summary>
        /// Omvandlar en float till byte[].
        /// </summary>
        /// <param name="argument"></param>
        /// <returns></returns>
        public static byte[] GetBytes(float argument)
        {
            return BitConverter.GetBytes(argument);
        }


        // Alla knappar skickar ett kommando till COM-porten.
        private void Forward_button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'W' };
                SendMove(command);
            }
        }

        private void Backward_button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'S' };
                SendMove(command);
            }
        }

        private void Right_button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'D' };
                SendMove(command);
            }
        }

        private void Left_Button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'A' };
                SendMove(command);
            }
        }
        private void Stop_button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'F' };
                SendMove(command);
            }
        }
        private void Rotate_Left_Button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'Q' };
                SendMove(command);
            }
        }
        private void Rotate_Right_Button_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { 'E' };
                SendMove(command);
            }
        }
        //Knappar för arm
        private void Platform_Left_Button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'T' };
                SendMove(command);
            }
        }
        private void Platform_Right_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'Y' };
                SendMove(command);
            }
        }
        private void Shoulder_Up_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'U' };
                SendMove(command);
            }
        }
        private void Shoulder_Down_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'I' };
                SendMove(command);
            }
        }
        private void Elbow_Up_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'G' };
                SendMove(command);
            }
        }
        private void Elbow_Down_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'H' };
                SendMove(command);
            }
        }
        private void Wrist_Up_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'J' };
                SendMove(command);
            }
        }
        private void Wrist_Down_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'K' };
                SendMove(command);
            }
        }
        private void Hand_Left_Button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'V' };
                SendMove(command);
            }
        }
        private void Hand_Right_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'B' };
                SendMove(command);
            }
        }
        private void Gripper_Hold_Button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'M' };
                SendMove(command);
            }
        }
        private void Gripper_Release_Button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'N' };
                SendMove(command);
            }
        }
        private void Turn_off_button_Click(object sender, RoutedEventArgs e)
        {
            char[] command = { 'P' };
            SendMove(command);
        }
        private void Start_position_button_Click(object sender, RoutedEventArgs e)
        {
            char[] command = { 'X' };
            SendMove(command);
        }
        private void Empty_Sample_Button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'O' };
                SendMove(command);
            }
        }
        private void Arm_speed_increase_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'u' };
                SendMove(command);
            }
        }
        private void Arm_speed_decrease_button_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 'd' };
                SendMove(command);
            }
        }
        private void Arm_mode_checked(object sender, RoutedEventArgs e)
        {
            char[] command = { 'a' };
            SendMove(command);
            arm = true;
            label_1.Content = "Rotation";
            label_2.Content = "V1";
            label_3.Content = "V2";
            label_4.Content = "V3";
            Angle1.Text = "";
            Angle2.Text = "";
            Angle3.Text = "";
            Angle4.Text = "";
            checkBox_1.IsEnabled = false;
            checkBox_2.IsEnabled = false;
            checkBox_3.IsEnabled = false;
            checkBox_4.IsEnabled = false;
            send = false;
        }
        private void Arm_mode_unchecked(object sender, RoutedEventArgs e)
        {
            char[] command = { 'b' };
            SendMove(command);
            arm = false;
            label_1.Content = "Speed";
            label_2.Content = "P";
            label_3.Content = "I";
            label_4.Content = "D";
            Angle1.Text = "";
            Angle2.Text = "";
            Angle3.Text = "";
            Angle4.Text = "";
            checkBox_1.IsEnabled = true;
            checkBox_2.IsEnabled = true;
            checkBox_3.IsEnabled = true;
            checkBox_4.IsEnabled = true;
        }
        private void Send_angles_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                receive_angle = true;
                char[] command = { 'v' };
                SendMove(command);
            }
        }
        private void Pick_up_Click(object sender, RoutedEventArgs e)
        {
            if (arm)
            {
                char[] command = { 't' };
                SendMove(command);
            }
        }


        /// <summary>
        /// Startar uppdrag och följer en linje tills slutet detekteras.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Start_mission_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { '7' };
                SendMove(command);
                mission = true;
            }
        }


        /// <summary>
        /// Vänder om och följer en linje tills starten detekteras.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Return_home_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                char[] command = { '8' };
                SendMove(command);
                mission = false;
            }
        }


        /// <summary>
        /// Knapp för att kalibrera linjesensorerna efter underlaget.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Calibrate_Click(object sender, RoutedEventArgs e)
        {
            if (!arm)
            {
                // Kalibrerar först för golv. Nästa gång knappen trycks kalibreras det för tejp.
                if (floor)
                {
                    char[] command = { '5' };
                    Calibrate.Content = "Tape";
                    SendMove(command);
                    floor = false;
                }
                else
                {
                    char[] command = { '6' };
                    Calibrate.Content = "Floor";
                    SendMove(command);
                    floor = true;
                }
            }
        }

        /// <summary>
        /// Skickar ut ett kommando i form av byte[] till COM-porten.
        /// </summary>
        /// <param name="command"></param>
        private void SendByte(byte[] command)
        {
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            if (inPort.IsOpen)
            {
                try
                {
                    inPort.Write(command, 0, 1);
                    textBox_Terminal.AppendText("[" + dtn + "] Sent: " + command[0] + "\n");
                    return;
                }
                catch (Exception e)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Timeout fail\n");
                }
            }
            textBox_Terminal.AppendText("[" + dtn + "] Tried to send " + command[0] + ", Port not open\n");
        }

        /// <summary>
        /// Skickar ut ett kommando i form av char[] till COM-porten.
        /// </summary>
        /// <param name="command"></param>
        private void SendMove(char[] command)
        {
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            if (inPort.IsOpen)
            {
                try
                {
                    inPort.Write(command, 0, 1);
                    textBox_Terminal.AppendText("[" + dtn + "] Sent: ");// + command[0] + "\n");
                    displayInfo(command[0]);
                    return;
                }
                catch (Exception e)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Timeout fail\n");
                }
            }
            textBox_Terminal.AppendText("[" + dtn + "] Tried to send: ");// + command[0] + "\n");
            displayInfo(command[0]);
            textBox_Terminal.AppendText("Port not open\n");
        }

        private void textBox_Terminal_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            textBox_Terminal.ScrollToEnd();
        }

        /// <summary>
        /// Tar hand om inkommande data på COM-porten.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void inPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (!inPort.IsOpen) return;

            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            bool check = false;
            this.Dispatcher.Invoke((Action)(() =>
            {
                check = checkBox_Hex.IsChecked.Value;
            }));

            if (receive_angle) // Ta emot vinklar
            {
                while (inPort.BytesToRead < 5) { }
           
                byte[] angles = new byte[5];
                inPort.Read(angles, 0, 5);
                //string output = BitConverter.ToString(angles);
                if (angles[0] == 0xFF)
                {
                     displayAngles(angles);
                }
               
                counter += 1;
                if (counter > 6)
                {
                    counter = 1;
                    receive_angle = false;
                }
                return;
            }

            if (check) // Visa hexadecimalt
            {
                byte[] bytes = new byte[inPort.BytesToRead]; 
                inPort.Read(bytes, 0, bytes.Length); 
                string output = BitConverter.ToString(bytes);
                this.Dispatcher.Invoke((Action)(() =>
                {
                    textBox_Terminal.AppendText("[" + dtn + "]" + output + "\n");
                }));
                return;
            }
            string message = string.Empty;

            try
            {
                message = inPort.ReadExisting();
                processMessage(message);
            }
            catch (IOException ex)
            {
                this.Dispatcher.Invoke((Action)(() =>
                {
                    textBox_Terminal.AppendText("[" + dtn + "]" + ex + "\n");
                }));
                return;
            }
        }

        /// <summary>
        /// Lägger på den mottagna datan i buffern och skriver ut information i terminalen 
        /// om buffern möter kraven ställda i buffer_containsStartChar
        /// </summary>
        /// <param name="message"></param>
        private void processMessage(string message)
        {
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            buffer += message;
            //buffer = Regex.Replace(buffer, @"\r\n?|\n", ""); // För att sålla bort PuTTy's newlines

            if (buffer.Length == 0) return;

            if (buffer_Processable())
            {
                this.Dispatcher.Invoke((Action)(() =>
                {
                textBox_Terminal.AppendText("[" + dtn + "] Recieved: ");// + buffer + "\n");
                }));

                currentInfo += buffer;
                buffer = string.Empty;

                this.Dispatcher.Invoke((Action)(() =>
                {
                    displayInfo(currentInfo[0]);
                }));
            }
            else if (buffer[0] != 'e') //Om inledande char indikerar ett kommando som vill vänta på mer information, bör vi inte tömma buffern.
            {
                this.Dispatcher.Invoke((Action)(() =>
                {
                textBox_Terminal.AppendText("[" + dtn + "] Recieved: " + buffer + "\n");
                }));
                buffer = string.Empty;
            }
        }

        /// <summary>
        /// Utifrån vilken första charen har för värde i buffern, så avgör denna funktion vilka ytterligare krav som ska ställas på indatan.
        /// </summary>
        /// <param></param>
        private bool buffer_Processable()
        {
            if (buffer.Length == 0)
            {
                return false;
            }
            char[] listofchars = {'a','b','u','d','W','A','D','S','Q','E','F','T','Y','U','I','G','H','J','K','V','B','N','M','X','P','O','5','6','7','8','s','v','t'};
            if (buffer[0] == 'e' && buffer.Length >= 5)
            {
                return true;
            }
            else if (listofchars.Contains(buffer[0]))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        /// <summary>
        /// Använder första char för att veta vad som behöver göras med informationen.
        /// </summary>
        /// <param></param>
        private void displayInfo(char c)
        {
            try
            {
                string text = "";
                if (c == 'e') //startbit för sensordata
                {
                    changeLamps();
                    text = "Sensor data\n";
                }
                else if (c == 'a')
                {
                    text = "Arm mode\n";
                }
                else if (c == 'b')
                {
                    text = "Drive mode\n";
                }
                else if (c == 'u')
                {
                    text = "Increase arm speed\n";
                }
                else if (c == 'd')
                {
                    text = "Decrease arm speed\n";
                }
                else if (c == 'W')
                {
                    text = "Drive forward\n";
                }
                else if (c == 'A')
                {
                    text = "Turn left\n";
                }
                else if (c == 'D')
                {
                    text = "Turn right\n";
                }
                else if (c == 'S')
                {
                    text = "Reverse\n";
                }
                else if (c == 'Q')
                {
                    text = "Rotate left\n";
                }
                else if (c == 'E')
                {
                    text = "Rotate right\n";
                }
                else if (c == 'F')
                {
                    text = "Stop\n";
                }
                else if (c == 'T')
                {
                    text = "Rotate platform left\n";
                }
                else if (c == 'Y')
                {
                    text = "Rotate platform right\n";
                }
                else if (c == 'U')
                {
                    text = "Shoulder up\n";
                }
                else if (c == 'I')
                {
                    text = "Shoulder down\n";
                }
                else if (c == 'G')
                {
                    text = "Elbow up\n";
                }
                else if (c == 'H')
                {
                    text = "Elbow down\n";
                }
                else if (c == 'J')
                {
                    text = "Wrist up\n";
                }
                else if (c == 'K')
                {
                    text = "Wrist down\n";
                }
                else if (c == 'V')
                {
                    text = "Hand left\n";
                }
                else if (c == 'B')
                {
                    text = "Hand right\n";
                }
                else if (c == 'N')
                {
                    text = "Release\n";
                }
                else if (c == 'M')
                {
                    text = "Hold\n";
                }
                else if (c == 'X')
                {
                    text = "Start position\n";
                }
                else if (c == 'P')
                {
                    text = "Turn off\n";
                }
                else if (c == 'O')
                {
                    text = "Empty sample\n";
                }
                else if (c == '5')
                {
                    text = "Calibrate floor\n";
                }
                else if (c == '6')
                {
                    text = "Calibrate tape\n";
                }
                else if (c == '7')
                {
                    text = "Start mission\n";
                }
                else if (c == 's')
                {
                    text = "Stop detected\n";
                }
                else if (c == 'v')
                {
                    text = "Send angles\n";
                }
                else if (c == 't')
                {
                    text = "Pick up\n";
                }
                else if (c == '8')
                {
                    text = "Return home\n";
                }
                else
                {
                    text = c + "\n";
                }

                this.Dispatcher.Invoke((Action)(() =>
                {
                    textBox_Terminal.AppendText(text);
                }));
                currentInfo = string.Empty;

                if (c == 's' && mission)
                {
                    Arm_mode.IsChecked = true;
                }

            }
            catch (Exception e)
            {
                this.Dispatcher.Invoke((Action)(() =>
                {
                    textBox_Terminal.AppendText("Error:" + e + "\n");
                }));
            }
        }

        //tar ut en bit ur en char
        int GetBitValue(char c, int bitPosition)
        {
            return ((c >> bitPosition) & 1);
        }


        /// <summary>
        /// Visar vinklarna som tagits emot.
        /// </summary>
        private void displayAngles(byte[] bytes)
        {
            string text = "Servo " + counter.ToString() + ": ";
            int angle = (bytes[3] << 8) | bytes[4];
            if (counter <= 4)
            {
                angle = (int)(angle * 0.08789);
            }
            else
            {
                angle = (int)(angle * 0.2935);
            }
            text += angle.ToString() + "\n";
            this.Dispatcher.Invoke((Action)(() =>
            {
                textBox_Terminal.AppendText(text);
            }));
        }

        /// <summary>
        /// Ändrar färg på "lamporna" utifrån strängen från currentInfo.
        /// </summary>
        /// <param></param>
        private void changeLamps()
        {
            //char på plats 1 innehåller sensordatan
            if ((GetBitValue(currentInfo[1], 0) == 1))
            {
                lamp1.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp1.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }
            if ((GetBitValue(currentInfo[1], 1) == 1))
            {
                lamp2.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp2.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }
            if ((GetBitValue(currentInfo[1], 2) == 1))
            {
                lamp3.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp3.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }
            if ((GetBitValue(currentInfo[1], 3) == 1))
            {
                lamp4.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp4.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }
            if ((GetBitValue(currentInfo[1], 4) == 1))
            {
                lamp5.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp5.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }
            if ((GetBitValue(currentInfo[1], 5) == 1))
            {
                lamp6.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp6.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }
            if ((GetBitValue(currentInfo[1], 6) == 1))
            {
                lamp7.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("Red");
            }
            else
            {
                lamp7.Fill = (SolidColorBrush)new BrushConverter().ConvertFromString("White");
            }   
        }

        private void SendStartbit()
        {
            byte[] command = { Convert.ToByte('!') };
            SendByte(command);
        }

        private void SendParameter(float parameter)
        {
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            char data = Convert.ToChar(parameter);
            char[] command = { data };

            if (inPort.IsOpen)
            {
                try
                {
                    SendMove(command);

                    textBox_Terminal.AppendText("[" + dtn + "] Sent: " + "P" + "\n");
                    return;
                }
                catch (Exception e)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Timeout fail\n");
                }
            }
            textBox_Terminal.AppendText("[" + dtn + "] Tried to send " + "P" + ", Port not open\n");

        }

        /// <summary>
        /// Tar in en uint32 och skickar iväg det som en array med bytes via en COM-port.
        /// </summary>
        /// <param name="number"></param>
        private void SendSpeed(UInt32 number)
        {
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            byte[] data = BitConverter.GetBytes(number);
            //char[] data = number.ToString().ToCharArray();

            if (inPort.IsOpen)
            {
                try
                {
                    inPort.Write(data, 0, 4);
                    textBox_Terminal.AppendText("[" + dtn + "] Sent: " + BitConverter.ToString(data) + "\n");
                    return;
                }
                catch (Exception e)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Timeout fail\n");
                }
            }
            textBox_Terminal.AppendText("[" + dtn + "] Tried to send " + BitConverter.ToString(data) + ", Port not open\n");
        }

        /// <summary>
        /// Tar in en float och skickar iväg det som en array med bytes via en COM-port.
        /// </summary>
        /// <param name="number"></param>
        private void SendValue(float number)
        {
            DateTime dt = DateTime.Now;
            string dtn = dt.ToShortTimeString();
            byte[] data = GetBytes(number);
            //char[] data = number.ToString().ToCharArray();

            if (inPort.IsOpen)
            {
                try
                {
                    inPort.Write(data, 0, 4);
                    textBox_Terminal.AppendText("[" + dtn + "] Sent: " + BitConverter.ToString(data) + "\n");
                    return;
                }
                catch (Exception e)
                {
                    textBox_Terminal.AppendText("[" + dtn + "] Timeout fail\n");
                }
            }
            textBox_Terminal.AppendText("[" + dtn + "] Tried to send " + BitConverter.ToString(data) + ", Port not open\n");
        }


        /// <summary>
        /// Hanterar tangentbordet.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            char[] command = { 'o' };
            switch (e.Key)
            {
                //Körläge
                case Key.W:
                    if (!arm)
                        command[0] = 'W'; //Framåt
                    break;
                case Key.A:
                    if (!arm)
                        command[0] = 'A'; //Vänster
                    break;
                case Key.D:
                    if (!arm)
                        command[0] = 'D'; //Höger
                    break;
                case Key.S:
                    if (!arm)
                        command[0] = 'S'; //Nedåt
                    break;
                case Key.F:
                    if (!arm)
                        command[0] = 'F'; //Stopp
                    break;
                case Key.Q:
                    if (!arm)
                        command[0] = 'Q'; //Rotera vänster
                    break;
                case Key.E:
                    if (!arm)
                        command[0] = 'E'; //Rotera höger
                    break;
                //Armläge
                case Key.T:
                    if (arm)
                        command[0] = 'T'; //Rotera plattform vänster
                    break;
                case Key.Y:
                    if (arm)
                        command[0] = 'Y'; //Rotera plattform höger
                    break;
                case Key.U:
                    if (arm)
                        command[0] = 'U'; //Axel uppåt
                    break;
                case Key.I:
                    if (arm)
                        command[0] = 'I'; //Axel nedåt
                    break;
                case Key.G:
                    if (arm)
                        command[0] = 'G'; //Armbåge uppåt
                    break;
                case Key.H:
                    if (arm)
                        command[0] = 'H'; //Armbåge nedåt
                    break;
                case Key.J:
                    if (arm)
                        command[0] = 'J'; //Handled uppåt
                    break;
                case Key.K:
                    if (arm)
                        command[0] = 'K'; //Handled nedåt
                    break;
                case Key.V:
                    if (arm)
                        command[0] = 'V'; //Hand vänster
                    break;
                case Key.B:
                    if (arm)
                        command[0] = 'B'; //Hand höger
                    break;
                case Key.N:
                    if (arm)
                        command[0] = 'N'; //Grepp släpp
                    break;
                case Key.M:
                    if (arm)
                        command[0] = 'M'; //Grepp håll
                    break;
                case Key.L:
                    if (arm)
                        command[0] = 't'; //Ta upp provrör
                    break;
                case Key.P:               //Byt mellan arm-läge och kör-läge
                    if (Arm_mode.IsChecked.Value)
                    {
                        Arm_mode.IsChecked = false;
                    }
                    else
                    {
                        Arm_mode.IsChecked = true;
                    } 
                     break;
            }

            if (command[0] != 'o')
            {
                SendMove(command);
            }
        }

        /*private void fabrik_button_Click(object sender, RoutedEventArgs e)
        {
            object result = null;
            double[] pos = { 0, 0, 0, 0.15, 0.141, 0.15, 0.296, 0.15 };
            double x = double.Parse(X_BOX.Text);
            double y = double.Parse(Y_BOX.Text);
            double[] t = { x, y };
            textBox_Terminal.AppendText("Sent: " + x + ","+ y + "\n");
            try
            {
                matlab.Feval("FABRIK", 2, out result, pos, t);

                object[] res = result as object[];

                double[,] new_pos = (double[,]) res[0];
                double[,] theta = (double[,]) res[1];

                string output = "Recieved:\nPositions:\n";
                foreach(double d in new_pos)
                {
                    output += Convert.ToString(d) + "\n";
                }

                output += "\nAngles:\n";
                foreach (double d in theta)
                {
                    output += Convert.ToString(d) + "\n";
                }
                
                    textBox_Terminal.AppendText(output + "\n");
                

        }
            catch (Exception ex)
            {
                this.Dispatcher.Invoke((Action)(() =>
                {
                    textBox_Terminal.AppendText("HELP: " + ex + "\n");
                }));*/

       /* private void Popup_button_Click(object sender, RoutedEventArgs e)
        {
            Popup popup = new Popup();
            System.Windows.Forms.DialogResult dialogresult = popup.ShowDialog();
            if (dialogresult == System.Windows.Forms.DialogResult.OK)
            {
                Console.WriteLine("You clicked OK");
            }
            else if (dialogresult == System.Windows.Forms.DialogResult.Cancel)
            {
                Console.WriteLine("You clicked either Cancel or X button in the top right corner");
            }
            popup.Dispose();
        }*/
    }
}
