using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.ComponentModel;
using ROM_Demo.Framework;
using System.Windows.Media.Media3D;
using System.Net.Sockets;
using System.Net;
using PQC.KinectMathHelpers;
using System.Threading;
using System.IO;

namespace ROM_Demo
{


    public partial class MainWindow : Window
    {//主类，窗口类
        #region Fields
        KinectSensor kinect;

        // Color Fields
        readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        ColorFrameReader colorReader;
        byte[] intermediaryArray;
        WriteableBitmap bitmap;

        // Body Fields
        BodyFrameReader bodyReader;
        Body[] bodies;
        CoordinateMapper coordinateMapper;
        DrawingGroup drawingGroup;
        int colorSpaceWidth;
        int colorSpaceHeight;

        // Brushes
        readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(150, 0, 255, 0));
        readonly Brush inferredJointBrush = new SolidColorBrush(Color.FromArgb(150, 255, 255, 0));
        readonly Pen trackedBonePen = new Pen(new SolidColorBrush(Color.FromArgb(150, 0, 255, 0)), 35);
        readonly Pen inferredBonePen = new Pen(new SolidColorBrush(Color.FromArgb(150, 255, 255, 0)), 35);
        #endregion
        //  public static bool SeverSendBackMassage = true;
        public static int count = 0;
        public string[] abc = new string[25] { "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0", };  //这个字符串用来记录角度。以后发送到服务器端
        //    public  string[] abc1 = new string[16];   //用于对数据进行平滑的三个字符串
        //    public string[] abc2 = new string[16];   //
        //    public string[] abc3 = new string[16];   //
        //    public string[] abc123 = new string[16];   //该字符串用于缓存上一次的输出

        public static int a = 0, b = 4;
        public string[,] abc6 = new string[5, 25];
        public string[] abc_out = new string[25];
        int n = 0;//计数器
        double tempR = 0, tempL = 0;
        public List<double> RRollList = new List<double>();
        public List<double> LRollList = new List<double>();
        public int Rlength = 0;
        public int Llength = 0;
        public const int angle = 20;//临界角度
        // public int i = 0;       //用于记录hip向里面弯的次数


        public static Socket socketClient = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);//sockets连接建立
        public static IPEndPoint endPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 8888);
        public static byte[] data = new byte[1024 * 1024];
        public static EndPoint Remote = (EndPoint)endPoint;

        /*   class Alpha
           {
               public void Beta()
               {
                   while (!SeverSendBackMassage)
                   {
                       //定义一个发送终结点，没有具体的IP和Port

                     socketClient.ReceiveFrom(data, ref Remote);
                       if (data.ToString() == "done")
                       {
                           SeverSendBackMassage = true;
                           Console.WriteLine("Sever had Sent Back 'done' to client");
                       }
                   }
               }
           }
         */

        public MainWindow()
        {
            Initializearray();
            InitializeComponent();
            //          Alpha oAlpha = new Alpha();
            //           Thread oThread = new Thread(new ThreadStart(oAlpha.Beta));
            //          oThread.Start();
            InitializeKinect();
        }
        void Initializearray()
        {
            RRollList.Add(0);
            LRollList.Add(0);
            do
            {
                LRollList.Add(LRollList[LRollList.Count - 1] - 0.5);
                RRollList.Add(RRollList[RRollList.Count - 1] + 0.5);//0.5为每帧改变的角度，可以改
            } while (RRollList[RRollList.Count - 1] < angle);
            if (RRollList[RRollList.Count - 1] > angle)
            {
                RRollList[RRollList.Count - 1] = angle;
                LRollList[LRollList.Count - 1] = -angle;
            }
        }

        void InitializeKinect()
        {                          //Kinect初始化
            kinect = KinectSensor.GetDefault();

            if (kinect == null)
            {
                MessageBox.Show("Kinect not found. Please reconnect device and try again.");
                this.Close();
            }

            kinect.Open();

            InitializeColorReading();
            InitializeBodyReading();
        }

        void InitializeColorReading()
        {                     //初始化读取Kinect 彩色帧
            var frameDescription = kinect.ColorFrameSource.FrameDescription;

            colorSpaceWidth = frameDescription.Width;
            colorSpaceHeight = frameDescription.Height;

            colorReader = kinect.ColorFrameSource.OpenReader();
            intermediaryArray = new byte[frameDescription.Width * frameDescription.Height * bytesPerPixel];
            bitmap = new WriteableBitmap(frameDescription.Width, frameDescription.Height, 96, 96, PixelFormats.Bgr32, null);
            ColorStreamImage.Source = bitmap;
        }

        void InitializeBodyReading()
        {                       //初始化读取Kinect  body帧
            coordinateMapper = kinect.CoordinateMapper;
            drawingGroup = new DrawingGroup();
            SkeletonStreamImage.Source = new DrawingImage(drawingGroup);

            var bodyFrameSource = kinect.BodyFrameSource;

            bodies = new Body[bodyFrameSource.BodyCount];
            bodyReader = kinect.BodyFrameSource.OpenReader();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {             //窗口初始化
            if (colorReader != null)
                colorReader.FrameArrived += ColorFrame_Arrived;
            if (bodyReader != null)
                bodyReader.FrameArrived += BodyFrame_Arrived;
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {           //关闭窗口事件触发
            if (colorReader != null)
            {
                colorReader.Dispose();
                colorReader = null;
            }
            if (bodyReader != null)
            {
                bodyReader.Dispose();
                bodyReader = null;
            }
            if (kinect != null)
            {
                kinect.Close();
                kinect = null;
            }
            socketClient.SendTo(Encoding.UTF8.GetBytes("quit"), endPoint);
        }

        void ColorFrame_Arrived(object sender, ColorFrameArrivedEventArgs e)
        {         //彩色帧获得 事件触发
            var frameReference = e.FrameReference;

            if (frameReference == null)
                return;

            var frame = frameReference.AcquireFrame();

            if (frame == null)
                return;

            using (frame)
            {
                var frameDescription = frame.FrameDescription;

                if (frameDescription.Width == bitmap.PixelWidth && frameDescription.Height == bitmap.PixelHeight)
                {
                    if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
                    {
                        frame.CopyRawFrameDataToArray(intermediaryArray);
                    }
                    else
                    {
                        frame.CopyConvertedFrameDataToArray(intermediaryArray, ColorImageFormat.Bgra);
                    }

                    bitmap.WritePixels(new Int32Rect(0, 0, frameDescription.Width, frameDescription.Height), intermediaryArray, (int)(frameDescription.Width * bytesPerPixel), 0);
                }
            }
        }

        void BodyFrame_Arrived(object sender, BodyFrameArrivedEventArgs e)
        {           //body帧获得事件触发  
            double closestBodyDistance = double.MaxValue;
            Body result = null;

            var frameReference = e.FrameReference;

            if (frameReference == null)
                return;

            var frame = frameReference.AcquireFrame();

            if (frame == null)
                return;

            //     data = Encoding.UTF8.GetBytes("hello,这里是通信模块的客户端，用来发送关节角度给服务器端，服务器端会控制nao的关节角度来进行动作");

            //将数据发送到服务器的终结点
            //      socketClient.SendTo(data, endPoint);

            //定义一个发送终结点，没有具体的IP和Port

            //定义一个网络地址
            EndPoint Remote = (EndPoint)endPoint;
            using (frame)
            {
                frame.GetAndRefreshBodyData(bodies);

                using (var dc = drawingGroup.Open())
                {
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, colorSpaceWidth, colorSpaceHeight));

                    foreach (var body in bodies)
                    {
                        if (body.IsTracked)
                        {
                            var currentLocation = body.Joints[JointType.SpineBase].Position;

                            var currentDistance = VectorLength(currentLocation);

                            if (result == null || currentDistance < closestBodyDistance)
                            {
                                result = body;
                                closestBodyDistance = currentDistance;
                            }
                            //画右手臂
                            var rShoulder = result.Joints[JointType.ShoulderRight];
                            var rElbow = result.Joints[JointType.ElbowRight];
                            var rWrist = result.Joints[JointType.HandRight];
                            DrawBone(rShoulder, rElbow, dc);
                            DrawBone(rElbow, rWrist, dc);

                            //画左手臂
                            var lShoulder = result.Joints[JointType.ShoulderLeft];
                            var lElbow = result.Joints[JointType.ElbowLeft];
                            var lWrist = result.Joints[JointType.HandLeft];
                            DrawBone(lShoulder, lElbow, dc);
                            DrawBone(lElbow, lWrist, dc);
                            //画右腿
                            var rhip = result.Joints[JointType.HipRight];
                            var rknee = result.Joints[JointType.KneeRight];
                            var rankle = result.Joints[JointType.AnkleRight];
                            DrawBone(rhip, rknee, dc);
                            DrawBone(rknee, rankle, dc);
                            //画左腿
                            var lhip = result.Joints[JointType.HipLeft];
                            var lknee = result.Joints[JointType.KneeLeft];
                            var lankle = result.Joints[JointType.AnkleLeft];
                            DrawBone(lhip, lknee, dc);
                            DrawBone(lknee, lankle, dc);
                            //画主干
                            var shoulder = result.Joints[JointType.SpineShoulder];
                            var spine = result.Joints[JointType.SpineMid];
                            var hip = result.Joints[JointType.SpineBase];
                            DrawBone(rShoulder, shoulder, dc);
                            DrawBone(lShoulder, shoulder, dc);
                            DrawBone(shoulder, spine, dc);
                            DrawBone(spine, hip, dc);
                            DrawBone(rhip, hip, dc);
                            DrawBone(lhip, hip, dc);
                            //	UpdateAngle(rShoulder, rElbow, rWrist);
                            GetAngles(result);
                            Sendtosever();

                            //    Receivefromthesever();
                        }
                    }

                }
            }
        }

        /* private static Body FindClosestBody(BodyFrame bodyFrame)
         {
             Body result = null;
             double closestBodyDistance = double.MaxValue;

             Body[] bodies = new Body[bodyFrame.BodyCount];
             bodyFrame.GetAndRefreshBodyData(bodies);

             foreach (var body in bodies)
             {
                 if (body.IsTracked)
                 {
                     var currentLocation = body.Joints[JointType.SpineBase].Position;

                     var currentDistance = VectorLength(currentLocation);

                     if (result == null || currentDistance < closestBodyDistance)
                     {
                         result = body;
                         closestBodyDistance = currentDistance;
                     }
                 }
             }

             return result;
         }
         * */
        private static double VectorLength(CameraSpacePoint point)
        {
            var result = Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2);

            result = Math.Sqrt(result);

            return result;
        }


        /*       private void Receivefromthesever()
               {
                   if (!SeverSendBackMassage)
                   {
                       Socket socketClient = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

                       //第二步 通过套接字收发报文

                       byte[] data = new byte[1024 * 1024];

                    data = Encoding.UTF8.GetBytes("test");

                       //将数据发送到服务器的终结点
                       socketClient.SendTo(data, endPoint);


                       //重新实例化一个字节数组 用于存放接受到的数据
                       data = new byte[1024 * 1024];
                       //接受数据 将数据保存到data数据 将远程主机的节点保存到Remote终端中【注意ref引用】
                       int receive = socketClient.ReceiveFrom(data, ref Remote);

                       Console.WriteLine("Message Receive From {0}", Remote.ToString());
                       Console.WriteLine(Encoding.UTF8.GetString(data, 0, receive));
                       SeverSendBackMassage = true;


                   }
               }
          */

        private void Sendtosever()                                //sockets发送角度数据给服务器端（C#这个程序为客户端），不过我们采用的是udp通信，两者是平等的
        {
            //if (count < 3)
            // {
            //    count++;
            // }
            //else
            //{
            string SendToServer;
            SendToServer = String.Join(" ", abc_out);
            //                     Console.WriteLine(SendToServer);
            socketClient.SendTo(Encoding.UTF8.GetBytes(SendToServer), Remote);

            Console.WriteLine("HeadPitch {0} HeadYaw {1} LShoulderRoll {2} LShoulderPitch {3} LElbowRoll {4} LElbowYaw {5} LWristYaw {6} RShoulderRoll {7} RShoulderPitch {8} RElbowRoll {9} RElbowYaw {10} RWristYaw {11} LHipRoll {12} LKnee {13} RHipRoll {14} RKnee {15}  LHandState {16} RHandState {17} LHipPitch {18} RHipPitch {19} LAnklePtich {20} RAnklePtich {21} HipYawPitch{22} LAnkleroll{23} RAnkleroll{24}", abc_out[0], abc_out[1], abc_out[2], abc_out[3], abc_out[4], abc_out[5], abc_out[6], abc_out[7], abc_out[8], abc_out[9], abc_out[10], abc_out[11], abc_out[12], abc_out[13], abc_out[14], abc_out[15], abc_out[16], abc_out[17], abc_out[18], abc_out[19], abc_out[20], abc_out[21], abc_out[22], abc_out[23], abc_out[24]);

            //    Console.WriteLine(" HeadPitch {0} HeadYaw {1} LHandState {2} RHandState {3}", abc[0], abc[1], abc[16], abc[17]);
            //        }

            count = 0;
            // }



        }



        /*    private bool IsChangedBigerThan10()                      //未引用的一个程序，之前写的，用于稳定节点用的
            {

                    bool ReturnBool = false;

                    for (int i = 0; i < 16; i++)
                    {
                        if (int.Parse(abc123[i]) - int.Parse(abc[i]) > 10)
                        {

                            ReturnBool = true;
                        }
                    }
                    return ReturnBool;

            }*/

        void DrawBone(Joint joint1, Joint joint2, DrawingContext dc)
        {                   //在WPF中画出骨骼线的程序
            if (joint1.TrackingState == TrackingState.NotTracked || joint2.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            var p1 = coordinateMapper.MapCameraPointToColorSpace(joint1.Position);
            var p2 = coordinateMapper.MapCameraPointToColorSpace(joint2.Position);

            if (joint1.TrackingState == TrackingState.Inferred || joint2.TrackingState == TrackingState.Inferred)
            {
                dc.DrawLine(inferredBonePen, new Point(p1.X, p1.Y), new Point(p2.X, p2.Y));
            }
            else
            {
                dc.DrawLine(trackedBonePen, new Point(p1.X, p1.Y), new Point(p2.X, p2.Y));
            }

            if (joint1.TrackingState == TrackingState.Tracked)
            {
                dc.DrawEllipse(trackedJointBrush, null, new Point(p1.X, p1.Y), 18.0, 18.0);
            }
            else
            {
                dc.DrawEllipse(inferredJointBrush, null, new Point(p1.X, p1.Y), 18.0, 18.0);
            }

            if (joint2.TrackingState == TrackingState.Tracked)
            {
                dc.DrawEllipse(trackedJointBrush, null, new Point(p2.X, p2.Y), 18.0, 18.0);
            }
            else
            {
                dc.DrawEllipse(inferredJointBrush, null, new Point(p2.X, p2.Y), 18.0, 18.0);
            }

        }


        /*
                void UpdateAngle(Joint rShoulder, Joint rElbow, Joint rWrist) {
                    if(rShoulder.TrackingState == TrackingState.NotTracked ||
                        rElbow.TrackingState == TrackingState.NotTracked ||
                        rWrist.TrackingState == TrackingState.NotTracked) {

                        return;
                    }

                    var shoulderPoint = coordinateMapper.MapCameraPointToColorSpace(rShoulder.Position);
                    var elbowPoint = coordinateMapper.MapCameraPointToColorSpace(rElbow.Position);
                    var wristPoint = coordinateMapper.MapCameraPointToColorSpace(rWrist.Position);

                    // Negate the Y component since the screen space starts at the top
                    var v1 = new Vec2f(shoulderPoint.X - elbowPoint.X, -(shoulderPoint.Y - elbowPoint.Y));
                    var v2 = new Vec2f(wristPoint.X - elbowPoint.X, -(wristPoint.Y - elbowPoint.Y));

                    v1.Normalize();
                    v2.Normalize();

                    int angle = (int)Math.Round(Math.Acos(v1.Dot(v2)) * (180 / Math.PI));
                    angle -= 180;
                    angle = -angle;

                    AngleTextBlock.Text = angle.ToString();
                }
         */
        private CameraSpacePoint CreateEndPoint(CameraSpacePoint startP, float[] vec)                //类，标定坐标
        {
            CameraSpacePoint point = new CameraSpacePoint();
            point.X = startP.X + vec[0];
            point.Y = startP.Y + vec[1];
            point.Z = startP.Z + vec[2];
            return point;
        }

        public double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)                  //计算向量的角度的程序，不过其实Vector3D自带一个计算角度的 程序
        {
            double dotProduct = 0.0;
            vectorA.Normalize();
            vectorB.Normalize();


            dotProduct = Vector3D.DotProduct(vectorA, vectorB);

            return (double)Math.Acos(dotProduct) / Math.PI * 180;
        }



        /*public double getangleroll(Vector3D VectorRef, Vector3D VectorAssist, Vector3D Joint)//获取四肢关节的roll角度
        {
            double componenty = 0.0, componentx = 0.0;
            double AngleRoll = 0.0;
            int a = 1;
            if (Vector3D.DotProduct(VectorAssist, Joint) < 0)
            {
                a = -1;
            }
            VectorRef.Normalize();
            VectorAssist.Normalize();
            componenty = Vector3D.DotProduct(Joint, VectorRef);
            componentx = Vector3D.DotProduct(Joint, VectorAssist);
            VectorRef = Vector3D.Multiply(componenty, VectorRef);
            VectorAssist = Vector3D.Multiply(componentx, VectorAssist);
            Vector3D NewJoint = Vector3D.Add(VectorRef, VectorAssist);
            AngleRoll = Vector3D.AngleBetween(NewJoint, VectorRef);
            AngleRoll = a * AngleRoll;
            return AngleRoll;
        }*/
        public double getanglepitch(Vector3D VectorRef, Vector3D VectorAssist, Vector3D Joint)//获取四肢关节的pitch角度
        {
            double Anglepitch = 0.0;
            Vector3D NewVector = Vector3D.CrossProduct(VectorRef, VectorAssist);
            Anglepitch = Vector3D.AngleBetween(NewVector, Joint);
            return Anglepitch;
        }

        public Vector3D getvector(Vector3D A, Vector3D B, Vector3D C)
        {
            A.Normalize();
            B.Normalize();
            double componenty = 0.0, componentx = 0.0;
            componentx = Vector3D.DotProduct(C, A);
            componenty = Vector3D.DotProduct(C, B);
            A = Vector3D.Multiply(A, componentx);
            B = Vector3D.Multiply(B, componenty);
            Vector3D NewJoint = Vector3D.Add(A, B);
            return NewJoint;
        }



        public void GetAngles(Body skeleton)                                                    //获取各个骨骼关节点的角度，目前下半身的角度已经获取，只是暂时还无法应用
        {

            #region  handstate
            switch (skeleton.HandRightState)
            {
                case HandState.Open:
                    abc[17] = "1";
                    break;
                case HandState.Closed:
                    abc[17] = "0";
                    break;
                case HandState.Lasso:
                    abc[17] = "0.5";
                    break;
                case HandState.Unknown:
                    abc[17] = "0";
                    break;
                case HandState.NotTracked:
                    abc[17] = "0";
                    break;
                default:
                    break;
            }

            switch (skeleton.HandLeftState)
            {
                case HandState.Open:
                    abc[16] = "1";
                    break;
                case HandState.Closed:
                    abc[16] = "0";
                    break;
                case HandState.Lasso:
                    abc[16] = "0.5";
                    break;
                case HandState.Unknown:
                    abc[16] = "0";
                    break;
                case HandState.NotTracked:
                    abc[16] = "0";
                    break;
                default:
                    break;
            }
            #endregion
            Vector3D Shoulder = new Vector3D(skeleton.Joints[JointType.SpineShoulder].Position.X, skeleton.Joints[JointType.SpineShoulder].Position.Y, skeleton.Joints[JointType.SpineShoulder].Position.Z);
            Vector3D RShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D LShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);
            Vector3D RElbow = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D LElbow = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
            Vector3D RWrist = new Vector3D(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y, skeleton.Joints[JointType.WristRight].Position.Z);
            Vector3D LWrist = new Vector3D(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y, skeleton.Joints[JointType.WristLeft].Position.Z);
            Vector3D RKnee = new Vector3D(skeleton.Joints[JointType.KneeRight].Position.X, skeleton.Joints[JointType.KneeRight].Position.Y, skeleton.Joints[JointType.KneeRight].Position.Z);
            Vector3D LKnee = new Vector3D(skeleton.Joints[JointType.KneeLeft].Position.X, skeleton.Joints[JointType.KneeLeft].Position.Y, skeleton.Joints[JointType.KneeLeft].Position.Z);
            Vector3D RAnkle = new Vector3D(skeleton.Joints[JointType.AnkleRight].Position.X, skeleton.Joints[JointType.AnkleRight].Position.Y, skeleton.Joints[JointType.AnkleRight].Position.Z);
            Vector3D LAnkle = new Vector3D(skeleton.Joints[JointType.AnkleLeft].Position.X, skeleton.Joints[JointType.AnkleLeft].Position.Y, skeleton.Joints[JointType.AnkleLeft].Position.Z);
            Vector3D Hip = new Vector3D(skeleton.Joints[JointType.SpineBase].Position.X, skeleton.Joints[JointType.SpineBase].Position.Y, skeleton.Joints[JointType.SpineBase].Position.Z);
            Vector3D RHip = new Vector3D(skeleton.Joints[JointType.HipRight].Position.X, skeleton.Joints[JointType.HipRight].Position.Y, skeleton.Joints[JointType.HipRight].Position.Z);
            Vector3D LHip = new Vector3D(skeleton.Joints[JointType.HipLeft].Position.X, skeleton.Joints[JointType.HipLeft].Position.Y, skeleton.Joints[JointType.HipLeft].Position.Z);
            Vector3D Spine = new Vector3D(skeleton.Joints[JointType.SpineMid].Position.X, skeleton.Joints[JointType.SpineMid].Position.Y, skeleton.Joints[JointType.SpineMid].Position.Z);

            /*
            foreach (BoneOrientation orientation in skeleton.BoneOrientations)
              {
                  // Display bone with Rotation using quaternion
                  DrawBonewithRotation(orientation.StartJoint, orientation.EndJoint, orientation.AbsoluteRotation.Quaternion);
                  // Display hierarchical rotation using matrix
                  DrawHierarchicalRotation(orientation.StartJoint, orientation.HierarchicalRotation.Matrix);
              }
            */
            double AngleRShoulderPitch = 0;//57.3=180除以π，相当于将弧度转化为角度的变量

            if (Vector3D.DotProduct(Vector3D.CrossProduct(Spine - Shoulder, Shoulder - RShoulder), (RElbow - RShoulder)) > 0)
            {

                AngleRShoulderPitch = -Vector3D.AngleBetween(Spine - Shoulder, RElbow - RShoulder);
                AngleRShoulderPitch += 90;

            }
            else
            {

                AngleRShoulderPitch = Vector3D.AngleBetween(Spine - Shoulder, RElbow - RShoulder);
                AngleRShoulderPitch += 90;
            }



            double AngleLShoulderPitch = 0;


            if (Vector3D.DotProduct(Vector3D.CrossProduct(Spine - Shoulder, Shoulder - LShoulder), (LElbow - LShoulder)) > 0)
            {

                AngleLShoulderPitch = Vector3D.AngleBetween(Spine - Shoulder, LElbow - LShoulder);
                AngleLShoulderPitch += 90;

            }
            else
            {

                AngleLShoulderPitch = -Vector3D.AngleBetween(Spine - Shoulder, LElbow - LShoulder);
                AngleLShoulderPitch += 90;
            }


            /*
            double AngleLHipPitch = 0;
            double AngleRHipPitch = 0;
            if (Vector3D.DotProduct(Vector3D.CrossProduct(RHip - Hip, Spine - Hip), (LKnee - LHip)) > 0)
            {
                AngleLHipPitch = 180 - (int)Vector3D.AngleBetween(Spine - Hip, LKnee - LHip);
            }
            else
            {
                AngleLHipPitch = (int)Vector3D.AngleBetween(Spine - Hip, LKnee - LHip) - 180;
            }
            if (Vector3D.DotProduct(Vector3D.CrossProduct(Spine - Hip, LHip - Hip), (RKnee - RHip)) > 0)
            {
                AngleRHipPitch = 180 - (int)Vector3D.AngleBetween(Spine - Hip, RKnee - RHip);
            }
            else
            {
                AngleRHipPitch = (int)Vector3D.AngleBetween(Spine - Hip, RKnee - RHip) - 180;
            }
            */
            //  double AngleLShoulderPitch = getanglepitch(Spine - Shoulder, LShoulder - Shoulder, LElbow - LShoulder) - 90;
            // double AngleRShoulderPitch = 90 - getanglepitch(Spine - Shoulder, RShoulder - Shoulder, RElbow - RShoulder);
            double AngleLHipPitch = getanglepitch(Hip - Spine, LHip - Hip, LKnee - LHip) - 90 - 8;
            double AngleRHipPitch = -getanglepitch(Hip - Spine, RHip - Hip, RKnee - RHip) + 90 - 8;
            // AngleLHipPitch = AngleLHipPitch * 1.5;
            // AngleRHipPitch = AngleRHipPitch * 1.5;
            /* if ((int)Vector3D.AngleBetween(Spine - Shoulder, RElbow - RShoulder) > 90)
             {
                 AngleRShoulderPitch -= 0;
             }

             if ((int)Vector3D.AngleBetween(Spine - Shoulder, LElbow - LShoulder) > 90)
             {
                 AngleLShoulderPitch -= 90;
             }*/






            // double AngleLShoulderPitch = 0;   //正负待定
            //  double AngleRShoulderPitch = 0;    //正负待定
            //double AngleLHipPitch = 0;
            // double AngleRHipPitch = 0;
            double AngleLShoulderPitch0 = 0;
            double AngleRShoulderPitch0 = 0;
            if (Vector3D.DotProduct((Spine - Shoulder), (LElbow - LShoulder)) > 0)
            {
                Vector3D NewVector = getvector(Vector3D.CrossProduct(Spine - Shoulder, LShoulder - Shoulder), Spine - Shoulder, (LElbow - LShoulder));
                AngleLShoulderPitch0 = getanglepitch(Spine - Shoulder, LShoulder - Shoulder, NewVector);

            }
            else
            {
                Vector3D NewVector = getvector(Vector3D.CrossProduct(Spine - Shoulder, LShoulder - Shoulder), Spine - Shoulder, (LElbow - LShoulder));
                AngleLShoulderPitch0 = -getanglepitch(Spine - Shoulder, LShoulder - Shoulder, NewVector);
            }
            if (Vector3D.DotProduct((Spine - Shoulder), (RElbow - RShoulder)) > 0)
            {
                Vector3D NewVector = getvector(Vector3D.CrossProduct(Spine - Shoulder, RShoulder - Shoulder), Spine - Shoulder, (LElbow - LShoulder));
                AngleRShoulderPitch0 = getanglepitch(Spine - Shoulder, Shoulder - RShoulder, NewVector);

            }
            else
            {
                Vector3D NewVector = getvector(Vector3D.CrossProduct(Spine - Shoulder, RShoulder - Shoulder), Spine - Shoulder, (LElbow - LShoulder));
                AngleRShoulderPitch0 = -getanglepitch(Spine - Shoulder, Shoulder - RShoulder, NewVector);

            }
            
            //写文件
           /* FileStream fs = new FileStream("H:\\ak.txt", FileMode.Append);
            StreamWriter sw = new StreamWriter(fs);
            //开始写入
            sw.WriteLine(AngleRShoulderPitch0+","+AngleLShoulderPitch0+","+AngleRShoulderPitch+","+AngleLShoulderPitch+"\n");  //新算法
            //sw.Write(",");
            //sw.Write();
            /*sw.Write(",");
            sw.Write();  //原先算法
            sw.Write(",");
            sw.Write();
            sw.Write("\n");
            //清空缓冲区
            sw.Flush();
            //关闭流
            sw.Close();
            fs.Close();*/


            /*
            if (Vector3D.DotProduct(Vector3D.CrossProduct(RHip - Hip, Spine - Hip), (LKnee - LHip)) > 0)
            {
                AngleLHipPitch = -getanglepitch(Hip - Spine, LHip - Hip, LKnee - LHip) + 90;
            }
            else
            {
                AngleLHipPitch = getanglepitch(Hip - Spine, LHip - Hip, LKnee - LHip) - 90;
            }
            if (Vector3D.DotProduct(Vector3D.CrossProduct(Spine - Hip, LHip - Hip), (RKnee - RHip)) > 0)
            {
                AngleRHipPitch = getanglepitch(Hip - Spine, RHip - Hip, RKnee - RHip) - 90;
            }
            else
            {
                AngleRHipPitch = -getanglepitch(Hip - Spine, RHip - Hip, RKnee - RHip) + 90;
            }*/




            double AngleLShoulderRoll = (int)AngleBetweenTwoVectors(LShoulder - Shoulder, LShoulder - LElbow) - 100;
            double AngleRShoulderRoll = 100 - (int)AngleBetweenTwoVectors(RShoulder - Shoulder, RShoulder - RElbow);
            //double AngleLShoulderRoll = getangleroll(Spine - Shoulder, LShoulder - Shoulder, LElbow - LShoulder) - 10;  //更改后的shoulderroll
            // double AngleRShoulderRoll = -getangleroll(Spine - Shoulder, RShoulder - Shoulder, RElbow - RShoulder) + 10;


            double AngleRElbowRoll = 180 - (int)AngleBetweenTwoVectors(RElbow - RShoulder, RElbow - RWrist);
            double AngleLElbowRoll = (int)AngleBetweenTwoVectors(LElbow - LShoulder, LElbow - LWrist) - 180;



            double AngleLElbowYaw = 0;
            double AngleRElbowYaw = 0;
            Vector3D crossProduct1 = new Vector3D();
            Vector3D crossProduct2 = new Vector3D();
            Vector3D crossProduct3 = new Vector3D();
            Vector3D crossProduct4 = new Vector3D();

            crossProduct1 = Vector3D.CrossProduct((RShoulder - LShoulder), (RShoulder - RElbow));
            crossProduct2 = Vector3D.CrossProduct((RElbow - RShoulder), (RElbow - RWrist));
            if (Vector3D.DotProduct(crossProduct1, (RWrist - RElbow)) < 0)
            {

                AngleRElbowYaw = (int)Vector3D.AngleBetween(crossProduct1, crossProduct2);

            }
            else
            {
                AngleRElbowYaw = -(int)Vector3D.AngleBetween(crossProduct1, crossProduct2);
            }

            crossProduct3 = Vector3D.CrossProduct((LShoulder - RShoulder), (LShoulder - LElbow));
            crossProduct4 = Vector3D.CrossProduct((LElbow - LShoulder), (LElbow - LWrist));

            if (Vector3D.DotProduct(crossProduct3, (LWrist - LElbow)) < 0)
            {
                AngleLElbowYaw = (int)Vector3D.AngleBetween(crossProduct3, crossProduct4);

            }
            else
            {
                AngleLElbowYaw = -(int)Vector3D.AngleBetween(crossProduct3, crossProduct4);
            }


            /*这一部分本来是针对手腕部分的横滚角进行计算的，然而，Kinect2采集的手腕横滚角还是很不稳定，因此无法很好的利用，本意是希望手张开以后，采集手部的大拇指，以大拇指作为旋转的长度计算旋转的角度的，但是还是不太稳定

                        double AngleLWristYaw=0;

                        if ((skeleton.Joints[JointType.ThumbLeft].Position.X - skeleton.Joints[JointType.HandLeft].Position.X) > 0)
                        {
                            if ((skeleton.Joints[JointType.ThumbLeft].Position.Y - skeleton.Joints[JointType.HandLeft].Position.Y) > 0)
                            {
                                AngleLWristYaw = -57.3 * (Math.Asin((skeleton.Joints[JointType.ThumbLeft].Position.X - skeleton.Joints[JointType.HandLeft].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbLeft], skeleton.Joints[JointType.HandLeft])));
                                AngleLElbowYaw = 0;
                            }
                            else
                            {
                                double ALWY = -180 + 57.3 * (Math.Asin((skeleton.Joints[JointType.ThumbLeft].Position.X - skeleton.Joints[JointType.HandLeft].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbLeft], skeleton.Joints[JointType.HandLeft])));
                                if (ALWY<-104.5)
                                {
                                    AngleLElbowYaw = ALWY+104.5;
                                    AngleLWristYaw = -104.5;
                                }
                            }

                        }
                        else
                        {
                            if ((skeleton.Joints[JointType.ThumbLeft].Position.Y - skeleton.Joints[JointType.HandLeft].Position.Y) > 0)
                            {
                                AngleLWristYaw = -57.3*(Math.Asin((skeleton.Joints[JointType.ThumbLeft].Position.X - skeleton.Joints[JointType.HandLeft].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbLeft], skeleton.Joints[JointType.HandLeft])));
                                AngleLElbowYaw = 0;
                            }
                            else
                            {
                                double ALWY = 180 + 57.3*(Math.Asin((skeleton.Joints[JointType.ThumbLeft].Position.X - skeleton.Joints[JointType.HandLeft].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbLeft], skeleton.Joints[JointType.HandLeft])));
                                if (ALWY>104.5)
                                 {
                                     AngleLElbowYaw = ALWY - 104.5;
                                     AngleLWristYaw = 104.5;
                                }
                            }
                        }


                        double AngleRWristYaw=0;
                        if ((skeleton.Joints[JointType.ThumbRight].Position.X - skeleton.Joints[JointType.HandRight].Position.X) > 0)
                        {
                            if ((skeleton.Joints[JointType.ThumbRight].Position.Y - skeleton.Joints[JointType.HandRight].Position.Y) > 0)
                            {
                                AngleRWristYaw = -57.3*(Math.Asin((skeleton.Joints[JointType.ThumbRight].Position.X - skeleton.Joints[JointType.HandRight].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbRight], skeleton.Joints[JointType.HandRight])));
                                AngleRElbowYaw = 0;
                            }
                            else
                            {
                                double ARWY = -180 + 57.3*(Math.Asin((skeleton.Joints[JointType.ThumbRight].Position.X - skeleton.Joints[JointType.HandRight].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbRight], skeleton.Joints[JointType.HandRight])));
                                if (ARWY<-104.5)
                                {
                                    AngleRElbowYaw = ARWY+ 104.5;
                                    AngleRWristYaw = -104.5;
                                }
                            }
                        }
                        else
                        {
                            if ((skeleton.Joints[JointType.ThumbRight].Position.Y - skeleton.Joints[JointType.HandRight].Position.Y) > 0)
                            {
                                AngleRWristYaw = -57.3*(Math.Asin((skeleton.Joints[JointType.ThumbRight].Position.X - skeleton.Joints[JointType.HandRight].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbRight], skeleton.Joints[JointType.HandRight])));
                                AngleRElbowYaw = 0;
                            }
                            else
                            {
                                double ARWY = 180 + 57.3*(Math.Asin((skeleton.Joints[JointType.ThumbRight].Position.X - skeleton.Joints[JointType.HandRight].Position.X) / LengthOfThumbs(skeleton.Joints[JointType.ThumbRight], skeleton.Joints[JointType.HandRight])));
                                if (ARWY > 104.5)
                                {
                                    AngleRElbowYaw = ARWY - 104.5;
                                    AngleRWristYaw = 104.5;
                                }
                            }
                        }

                        */

            double AngleRKnee = 180 - (int)AngleBetweenTwoVectors(RKnee - RHip, RKnee - RAnkle);
            double AngleLKnee = 180 - (int)AngleBetweenTwoVectors(LKnee - LHip, LKnee - LAnkle);
            double AngleRHipRoll = 0;
            double AngleLHipRoll = 0;
            AngleLHipRoll = 90 - AngleBetweenTwoVectors(LHip - Hip, LKnee - LHip);
            AngleRHipRoll = AngleBetweenTwoVectors(RHip - Hip, RKnee - RHip) - 90;
            double AnkleLRoll = 0, AnkleRRoll = 0;
            /*  if(Vector3D.DotProduct (LHip -Hip ,LKnee-LHip )>0)
              {
                  AngleLHipRoll = 90-AngleBetweenTwoVectors(LHip - Hip, LKnee - LHip); 
              }
              else
              {
                  AngleLHipRoll = AngleBetweenTwoVectors(LHip - Hip, LKnee - LHip)-90; 
              }
              if (Vector3D.DotProduct(RHip - Hip, RKnee - RHip) > 0)
              {
                  AngleRHipRoll = AngleBetweenTwoVectors(RHip - Hip, RKnee - RHip)-90;
              }
              else
              {
                  AngleRHipRoll = 90-AngleBetweenTwoVectors(RHip - Hip, RKnee - RHip);
              }*/
            // double AngleRHipRoll =( -(int)getangleroll(Hip - Spine, RHip - Hip, RKnee - RHip) );//+5
            // double AngleLHipRoll = ((int)getangleroll(Hip - Spine, LHip - Hip, LKnee - LHip)) ;//-5

            if (AngleRHipRoll > 0)
            {
                AngleRHipRoll = AngleRHipRoll * 4;
                // if (AngleRHipRoll > 13 & AngleRHipRoll < 15)
                //    AngleRHipRoll = 15;
                // if (AngleRHipRoll > 15 & AngleRHipRoll <= 18)
                //    AngleRHipRoll = 17;
                if (AngleRHipRoll > angle)// & AngleRHipRoll < 25)
                    AngleRHipRoll = angle;
                //  if (AngleRHipRoll > 25 )
                //     AngleRHipRoll = 21;

            }
            if (AngleLHipRoll < 0)
            {
                AngleLHipRoll = AngleLHipRoll * 4;
                // if (AngleLHipRoll < -13 & AngleLHipRoll > -15)
                //     AngleLHipRoll = -15;
                // if (AngleLHipRoll < -15 & AngleLHipRoll >= -18)
                //      AngleLHipRoll = -17;
                if (AngleLHipRoll < -angle)//& AngleLHipRoll > -25)
                    AngleLHipRoll = -angle;
                // if (AngleLHipRoll <- 25 )
                //    AngleLHipRoll = -21;

            }
            if (n < 10)
            {
                n++;
                AngleRHipRoll = tempR;
                AngleLHipRoll = tempL;
            }
            else
            {
                tempL = AngleLHipRoll;
                tempR = AngleRHipRoll;
                n = 0;
            }
            AnkleLRoll = -AngleLHipRoll;
            AnkleRRoll = -AngleRHipRoll;
            if (AnkleLRoll > 19)
            {
                AnkleLRoll = 19;
            }
            if (AnkleRRoll < -19)
            {
                AnkleRRoll = -19;
            }
            // AngleRHipRoll = AngleRHipRoll * 2.5;
            //  AngleLHipRoll = AngleLHipRoll * 2.5;
            #region  judgement
            if (AngleLShoulderRoll > 76 || AngleLShoulderRoll < -18)
            {
                if (AngleLShoulderRoll > 76)
                {
                    AngleLShoulderRoll = 76;
                }
                else
                {
                    AngleLShoulderRoll = -18;
                }
            }
            if (Math.Abs(AngleLShoulderPitch0) > 119.5)     //将新算法的LShoulderRoll传给NAO
            {
                if (AngleLShoulderPitch0 > 0)
                {
                    AngleLShoulderPitch0 = 119.5;
                }
                else
                {
                    AngleLShoulderPitch0 = -119.5;
                }
            }
            if (AngleLElbowRoll > -2 || AngleLElbowRoll < -88.5)
            {
                if (AngleLElbowRoll > -2)
                {
                    AngleLElbowRoll = -2;
                }
                else
                {
                    AngleLElbowRoll = -88.5;
                }
            }
            if (AngleLElbowYaw > 119.5 || AngleLElbowYaw < -119.5)
            {
                if (AngleLElbowYaw > 119.5)
                {
                    AngleLElbowYaw = 119.5;
                }
                else
                {
                    AngleLElbowYaw = -119.5;
                }
            }
            /*
                      if (AngleLWristYaw > 104.5 || AngleLWristYaw < -104.5)
                      {
                          if (AngleLWristYaw > 104.5)
                          {
                              AngleLWristYaw = 104.5;
                          }
                          else
                          {
                              AngleLWristYaw = -104.5;
                          }
                      }
               */
            if (AngleRShoulderRoll > 18 || AngleRShoulderRoll < -76)
            {
                if (AngleRShoulderRoll > 18)
                {
                    AngleRShoulderRoll = 18;
                }
                else
                {
                    AngleRShoulderRoll = -76;
                }
            }
            if (AngleRShoulderPitch > 119.5 || AngleRShoulderPitch < -119.5)
            {
                if (AngleRShoulderPitch > 119.5)
                {
                    AngleRShoulderPitch = 119.5;
                }
                else
                {
                    AngleRShoulderPitch = -119.5;
                }
            }
            if (AngleRElbowRoll > 88.5 || AngleRElbowRoll < 2)
            {
                if (AngleRElbowRoll > 88.5)
                {
                    AngleRElbowRoll = 88.5;
                }
                else
                {
                    AngleRElbowRoll = 2;
                }
            }
            if (AngleRElbowYaw > 119.5 || AngleRElbowYaw < -119.5)
            {
                if (AngleRElbowYaw > 119.5)
                {
                    AngleRElbowYaw = 119.5;
                }
                else
                {
                    AngleRElbowYaw = -119.5;
                }
            }
            /*
                     if (AngleRWristYaw > 104.5|| AngleRElbowYaw < -104.5)
                     {
                         if (AngleRElbowYaw > 104.5)
                         {
                             AngleRElbowYaw = 104.5;
                         }
                         else
                         {
                             AngleRElbowYaw = -104.5;
                         }
                     }
             */
            /* if (AngleLHipRoll > 44.5 || AngleLHipRoll < -24.5)
             {
                 if (AngleLHipRoll > 44.5)
                 {
                     AngleLHipRoll = 44.5;
                 }
                 else
                 {
                     AngleLHipRoll = -24.5;
                 }
             }*/
            /* if (AngleRHipRoll > 24.5 || AngleRHipRoll < -44.5)
             {
                 if (AngleRHipRoll > 24.5)
                 {
                     AngleRHipRoll = 24.5;
                 }
                 else
                 {
                     AngleRHipRoll = -44.5;
                 }
             }*/
            if (AngleLKnee > 121.04 || AngleLKnee < -5.29)
            {
                if (AngleLKnee > 121.04)
                {
                    AngleLKnee = 121.04;
                }
                else
                {
                    AngleLKnee = -5.29;
                }
            }
            if (AngleRKnee > 121.04 || AngleRKnee < -5.29)
            {
                if (AngleRKnee > 121.04)
                {
                    AngleRKnee = 121.04;
                }
                else
                {
                    AngleRKnee = -5.29;
                }
            }
            if (AngleRHipPitch > 27 || AngleRHipPitch < -88)
            {
                if (AngleRHipPitch > 27)
                {
                    AngleRHipPitch = 27;
                }
                else
                {
                    AngleRHipPitch = -88;
                }
            }
            if (AngleLHipPitch > 27 || AngleLHipPitch < -88)
            {
                if (AngleLHipPitch > 27)
                {
                    AngleLHipPitch = 27;
                }
                else
                {
                    AngleLHipPitch = -88;
                }
            }
            if (AngleRHipRoll > 0)
            {
                AngleRHipRoll = SmoothRHipRoll(AngleRHipRoll);
            }
            if (AngleLHipRoll < 0)
            {
                AngleLHipRoll = SmoothLHipRoll(AngleLHipRoll);
            }

            #endregion

            result1.Text = AngleRElbowRoll.ToString();
            result2.Text = AngleRShoulderRoll.ToString();
            result3.Text = AngleLElbowRoll.ToString();
            result4.Text = AngleLShoulderRoll.ToString();
            result5.Text = AngleRKnee.ToString();
            result6.Text = AngleLKnee.ToString();
            result7.Text = AngleRHipRoll.ToString();
            result8.Text = AngleLHipRoll.ToString();
            //relbowyaw.Text = AngleRElbowYaw.ToString();           (临时修改显示界面)
            //lelbowyaw.Text = AngleLElbowYaw.ToString();

           // result9.Text = AngleLHipPitch.ToString();         result9-11 & elbowyaw
           // result10.Text = AngleRHipPitch.ToString();
         //   result13.Text = AngleLShoulderPitch.ToString();
          //  result14.Text = AngleRShoulderPitch.ToString();

            GetAnglesofHead(abc, skeleton);
            abc[2] = ((int)AngleLShoulderRoll).ToString();
            abc[3] = ((int)AngleLShoulderPitch0).ToString();        //新算法LShoulderPitch传值给NAO，共两处改动
            abc[4] = ((int)AngleLElbowRoll).ToString();
            //   abc[5] = (-(int)AngleLElbowYaw).ToString();
            //  abc[6] = (-(int)AngleLWristYaw).ToString();
            abc[5] = ((int)AngleLElbowYaw).ToString();
            abc[6] = "0";
            abc[7] = ((int)AngleRShoulderRoll).ToString();
            abc[8] = ((int)AngleRShoulderPitch).ToString();
            abc[9] = ((int)AngleRElbowRoll).ToString();
            //    abc[10] = (-(int)AngleRElbowYaw).ToString();
            //    abc[11] = (-(int)AngleRWristYaw).ToString();
            abc[10] = ((int)AngleRElbowYaw).ToString();
            abc[11] = "0";
            abc[12] = ((int)AngleLHipRoll).ToString();
            abc[13] = ((int)AngleLKnee).ToString();
            abc[14] = ((int)AngleRHipRoll).ToString();//rhiproll
            abc[15] = ((int)AngleRKnee).ToString();
            //abc[12] = "0";
            //abc[14] = "0";

            abc[18] = ((int)AngleLHipPitch).ToString();
            abc[19] = ((int)AngleRHipPitch).ToString();
            double a = 2.0;
            abc[20] = ((int)(-AngleLKnee / a)).ToString();//lanklepitch
            abc[21] = ((int)(-AngleRKnee / a)).ToString();//ranklepitch
            // abc[20] = ((int)(-(AngleLKnee + AngleLHipPitch) / 3)).ToString();//lanklepitch
            //abc[21] = ((int)(-(AngleRKnee + AngleRHipPitch) / 3)).ToString();//ranklepitch
            //result11.Text = abc[20];
           // result12.Text = abc[21];


            abc[23] = (-(int)AngleLHipRoll).ToString();
            abc[24] = (-(int)AngleRHipRoll).ToString();

            // abc[23] = "0";
            //abc[24] = "0";


            medianfilter(abc);
            /* 
                        bool IsEnabled = true;//这个变量的用处是判断缓冲区的三个字符串是否有空值，如果都没有的话就进行平均,然后才能进行比对，也就是说这个变量是进行比对之前的检测变量

                       for (int i = 0; i < 16; i++)
                        {
                            if (abc2[i] != null)
                            {
                                abc3[i] = abc2[i];
                            }
                            else
                            {
                                IsEnabled = false;
                            }
                            if (abc1[i] != null)
                            {
                                abc2[i] = abc1[i];
                            }
                            else
                            {
                                IsEnabled = false;
                            }
                            if (abc[i] != null)
                            {
                                abc1[i] = abc[i];
                            }
                            else
                            {
                                IsEnabled = false;
                            }
                        }
                        if (!IsEnabled)
                        {
                            for (int i = 0; i < 16; i++)
                            {
                                abc123[i] = abc[i];
                            }

                        }
                        if (IsEnabled)
                        {
                            for (int i = 0; i < 16; i++)
                            {
                               abc[i]=((int.Parse(abc1[i])+int.Parse(abc2[i])+int.Parse(abc3[i]))/3).ToString();
                            }

                        }
            */

            //  double[] resultsArray = Results.ToArray();

            //   return resultsArray;

        }
        private void medianfilter(string[] abc)          //中值滤波器，作用于关节角度
        {
            if (a < 4)
            {
                for (int i = 0; i < 25; ++i)
                {
                    abc6[a, i] = abc[i];
                }
                a++;

            }
            else
            {
                for (int i = 0; i < 25; ++i)
                {
                    abc6[b, i] = abc[i];
                    string[] array = { abc6[0, i], abc6[1, i], abc6[2, i], abc6[3, i], abc6[4, i] };
                    Array.Sort(array);
                    abc_out[i] = array[2];
                }

                b++;
                if (b > 4) b = 0;
            }

        }

        private double SmoothRHipRoll(double RRoll)
        {
            if ((RRoll - RRollList[Rlength]) > 0.5)
            {
                return RRollList[++Rlength];
            }
            else
            {
                if ((RRollList[Rlength] - RRoll) > 0.5)
                {
                    return RRollList[--Rlength];
                }
                else
                {
                    return RRoll;
                }
            }
        }
        private double SmoothLHipRoll(double LRoll)
        {
            if ((LRoll - LRollList[Llength]) < -0.5)
            {
                return LRollList[++Llength];
            }
            else
            {
                if ((LRoll - LRollList[Llength]) > 0.5)
                {
                    return LRollList[--Llength];
                }
                else
                {
                    return LRoll;
                }
            }
        }
        /*
        //R为正数
        private double SmoothRHipRoll(double RRoll)     
        {
            if (RRollList.Count == 0)
            {
                RRollList[0] = 0;
            }
            if (RRollList[Rlength] == angle)
            {
                Rlength = 0;
                RRollList.Clear();
                //RRollList[0] = RRoll;
                return RRoll;
            }
            if (angle == RRoll)
            {
                return RRoll;
            }
            //int i = 0;
            do
            {
                RRollList.Add(RRollList[RRollList.Count] + 0.5);//0.5为每帧改变的角度，可以改
            } while (RRollList[RRollList.Count - 1] < RRoll);
            if (RRollList[RRollList.Count - 1] >= RRoll)
            {
                RRollList[RRollList.Count - 1] = RRoll;
            }
            //Rlength++;
            return RRollList[Rlength++];
        }
        //L为负数
        private double SmoothLHipRoll(double LRoll)     //改动的地方为此处、函数的调用、变量的声明
        {
            if (LRollList.Count == 0)
            {
                LRollList[0] = 0;
            }
            if (LRollList[Llength] == -angle)
            {
                Llength = 0;
                LRollList.Clear();
                //LRollList[0] = LRoll;
                return LRoll;
            }
            if (-angle == LRoll)                        //-运算优先级可能会导致出错
            {
                return LRoll;
            }
            //int i = 0;
            do
            {
                LRollList.Add(LRollList[LRollList.Count] - 0.5);//0.5为每帧改变的角度，可以改
            } while (LRollList[LRollList.Count - 1] > LRoll);
            if (LRollList[LRollList.Count - 1] <= LRoll)
            {
                LRollList[LRollList.Count - 1] = LRoll;
            }
            //Llength++;
            return RRollList[Llength++];
        }
        */

        private void GetAnglesofHead(string[] abc, Body skeleton)                       //获取头部角度的程序，目前只有头部的角度是采用的是Kinect提取出的四元数的角度计算的，因此是单独计算的
        {
            double AngleBetweenParentChildY = 0;
            double AngleBetweenParentChildX = 0;

            Vector4 vecNeck = skeleton.JointOrientations[JointType.Neck].Orientation;
            PQC.KinectMathHelpers.Quaternion SpineShoulderOrientation = new PQC.KinectMathHelpers.Quaternion(vecNeck.W, vecNeck.X, vecNeck.Y, vecNeck.Z);
            CameraSpacePoint csNeckX = CreateEndPoint(skeleton.Joints[JointType.Neck].Position, SpineShoulderOrientation.Rotate(0.1f, 0.0f, 0.0f));
            CameraSpacePoint csNeckY = CreateEndPoint(skeleton.Joints[JointType.Neck].Position, SpineShoulderOrientation.Rotate(0.0f, 0.1f, 0.0f));
            CameraSpacePoint csNeckZ = CreateEndPoint(skeleton.Joints[JointType.Neck].Position, SpineShoulderOrientation.Rotate(0.0f, 0.0f, 0.1f));
            DepthSpacePoint dsNeckX = this.coordinateMapper.MapCameraPointToDepthSpace(csNeckX);
            DepthSpacePoint dsNeckY = this.coordinateMapper.MapCameraPointToDepthSpace(csNeckY);
            DepthSpacePoint dsNeckZ = this.coordinateMapper.MapCameraPointToDepthSpace(csNeckZ);
            Vector4 vecParentofNeck = skeleton.JointOrientations[JointType.SpineShoulder].Orientation;
            PQC.KinectMathHelpers.Quaternion qOrientationParentofNeck = new PQC.KinectMathHelpers.Quaternion(vecParentofNeck.W, vecParentofNeck.X, vecParentofNeck.Y, vecParentofNeck.Z);
            CameraSpacePoint csXParent = CreateEndPoint(skeleton.Joints[JointType.SpineShoulder].Position, qOrientationParentofNeck.Rotate(0.1f, 0.0f, 0.0f));
            DepthSpacePoint dsXParent = this.coordinateMapper.MapCameraPointToDepthSpace(csXParent);
            AngleBetweenParentChildX = MathHelpers.AngleBetweenPoints(new Point(dsNeckX.X, dsNeckX.Y), new Point(dsXParent.X, dsXParent.Y));

            abc[0] = ((int)AngleBetweenParentChildX + 90).ToString();

            //           CameraSpacePoint csYParent = CreateEndPoint(skeleton.Joints[ JointType.SpineShoulder].Position, qOrientationParentofNeck.Rotate(0.0f, 0.1f, 0.0f));
            //           DepthSpacePoint dsYParent = this.coordinateMapper.MapCameraPointToDepthSpace(csYParent);
            //           AngleBetweenParentChildY = MathHelpers.AngleBetweenPoints(new Point(dsNeckY.X, dsNeckY.Y), new Point(dsYParent.X, dsYParent.Y));


            //         CameraSpacePoint csZParent = CreateEndPoint(skeleton.Joints[ JointType.SpineShoulder].Position, qOrientationParentofNeck.Rotate(0.0f, 0.0f, 0.1f));
            //         DepthSpacePoint dsZParent = this.coordinateMapper.MapCameraPointToDepthSpace(csZParent);
            //         AngleBetweenParentChildZ = MathHelpers.AngleBetweenPoints(new Point(dsNeckZ.X, dsNeckY.Y), new Point(dsZParent.X, dsZParent.Y));

            Vector4 vecHead = skeleton.JointOrientations[JointType.Head].Orientation;
            PQC.KinectMathHelpers.Quaternion qOrientation = new PQC.KinectMathHelpers.Quaternion(vecHead.W, vecHead.X, vecHead.Y, vecHead.Z);
            CameraSpacePoint csHeadX = CreateEndPoint(skeleton.Joints[JointType.Head].Position, qOrientation.Rotate(0.1f, 0.0f, 0.0f));
            CameraSpacePoint csHeadY = CreateEndPoint(skeleton.Joints[JointType.Head].Position, qOrientation.Rotate(0.0f, 0.1f, 0.0f));
            CameraSpacePoint csHeadZ = CreateEndPoint(skeleton.Joints[JointType.Head].Position, qOrientation.Rotate(0.0f, 0.0f, 0.1f));
            DepthSpacePoint dsHeadX = this.coordinateMapper.MapCameraPointToDepthSpace(csHeadX);
            DepthSpacePoint dsHeadY = this.coordinateMapper.MapCameraPointToDepthSpace(csHeadY);
            DepthSpacePoint dsHeadZ = this.coordinateMapper.MapCameraPointToDepthSpace(csHeadZ);
            Vector4 vecParentofHead = skeleton.JointOrientations[JointType.Neck].Orientation;
            PQC.KinectMathHelpers.Quaternion qOrientationParentofHead = new PQC.KinectMathHelpers.Quaternion(vecParentofHead.W, vecParentofHead.X, vecParentofHead.Y, vecParentofHead.Z);
            CameraSpacePoint csYParent = CreateEndPoint(skeleton.Joints[JointType.Neck].Position, qOrientationParentofHead.Rotate(0.0f, 0.1f, 0.0f));
            DepthSpacePoint dsYParent = this.coordinateMapper.MapCameraPointToDepthSpace(csYParent);
            AngleBetweenParentChildY = MathHelpers.AngleBetweenPoints(new Point(dsHeadY.X, dsHeadY.Y), new Point(dsYParent.X, dsYParent.Y));

            abc[1] = ((int)AngleBetweenParentChildY + 90).ToString();
            //           CameraSpacePoint csZParent = CreateEndPoint(skeleton.Joints[JointType.Neck].Position, qOrientationParentofHead.Rotate(0.0f, 0.0f, 0.1f));
            //          DepthSpacePoint dsZParent = this.coordinateMapper.MapCameraPointToDepthSpace(csZParent);
            //          AngleBetweenParentChildZ = MathHelpers.AngleBetweenPoints(new Point(dsNeckZ.X, dsNeckY.Y), new Point(dsZParent.X, dsZParent.Y));



        }



        private double LengthOfThumbs(Joint joint1, Joint joint2)
        {

            double b = Math.Pow(joint1.Position.X - joint2.Position.X, 2) + Math.Pow(joint1.Position.Y - joint2.Position.Y, 2);
            return Math.Sqrt(b);
            throw new NotImplementedException();
        }

        private double LengthBetweenJoints(Joint joint1, Joint joint2)
        {
            double a = Math.Pow(joint1.Position.X - joint2.Position.X, 2) + Math.Pow(joint1.Position.Y - joint2.Position.Y, 2) + Math.Pow(joint1.Position.Z - joint2.Position.Z, 2);
            return Math.Sqrt(a);
            throw new NotImplementedException();
        }


    }
}
/*



*/
