using System;
using System.IO.Ports;
using System.Windows.Forms;
using System.Threading;
using System.Threading.Tasks;

using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.Util;
using Emgu.CV.UI;


namespace deltabot1_interface
{
 
    public partial class Form1 : Form
    {
        

        String[] ports;
        SerialPort port;
        // robot geometry
        // (look at pics above for explanation)
        Double ee = 173.205;     // end effector
        Double ff = 346.4;// base
        Double re = 400;
        Double rf = 170;

        // trigonometric constants
        const Double sqrt3 = 1.732;
        const Double pi = 3.1415;    // PI
        const Double sin120 = 0.866;
        const Double cos120 = -0.5;
        const Double tan60 = 1.732;
        const Double sin30 = 0.5;
        const Double tan30 = 0.577;
        int fps = 10;
        VideoCapture Capture;
        bool Pause = false;
        bool Fetch = false;


        public Form1()
        {
            InitializeComponent();
            
            ports = SerialPort.GetPortNames();
            foreach (string port in ports)
            {
                comboBox1.Items.Add(port);
                Console.WriteLine(port);
                if (ports[0] != null)
                {
                    comboBox1.SelectedItem = ports[0];
                }
            }
            disableControls();


        }
        public void Parameters(Double e1,Double f1,Double re1,Double rf1)
        {
            ee = e1;
            ff = f1;
            re = re1;
            rf = rf1;

           textBox8.Text = ("- e=" + (Convert.ToString(ee)) + ";f=" + (Convert.ToString(ff)) + ";re=" + (Convert.ToString(re)) + ";rf=" + (Convert.ToString(rf)));
           

        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {



        }

        private void button1_Click(object sender, EventArgs e)
        {

            string selectedPort = comboBox1.GetItemText(comboBox1.SelectedItem);
            port = new SerialPort(selectedPort, 115200);
            port.Open();
            port.Write("\r\n\r\n");
            enableControls();
            button1.Enabled = false;


        }

        private void enableControls()
        {
            textBox1.Enabled = true;
            button2.Enabled = true;

        }
        private void disableControls()
        {
            textBox1.Enabled = false;
            button2.Enabled = false;

        }



        private void button2_Click(object sender, EventArgs e)
        {

            port.WriteLine(textBox1.Text);

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void button4_Click(object sender, EventArgs e)
        {

            // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
            // returned status: 0=OK, -1=non-existing position
            Double T1 = Convert.ToDouble(textBox5.Text);
            Double T2 = Convert.ToDouble(textBox6.Text);
            Double T3 = Convert.ToDouble(textBox7.Text);
            Double T12 = T1 ;
            Double T22 = T2 ;
            Double T32 = T3 ;
            Double X1 = 0;
            Double Y1 = 0;
            Double Z1 = 0;

            int Status1 = delta_calcForward(T1, T2, T3,ref X1,ref Y1,ref Z1);
            if (Status1 == 0)
            {
                textBox8.Text = "OK";
                textBox2.Text = Convert.ToString(X1);
                textBox3.Text = Convert.ToString(Y1);
                textBox4.Text = Convert.ToString(Z1);
            }
            else
            {
                textBox8.Text = "non-existing position";
                textBox2.Text = " Ø ";
                textBox3.Text = " Ø ";
                textBox4.Text = " Ø ";

            }

            

        }
        // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
        // returned status: 0=OK, -1=non-existing position
        int delta_calcForward(Double theta1, Double theta2, Double theta3, ref Double x0,ref Double y0,ref Double z0)
        {
            Double t = (ff - ee) * tan30 / 2;
            Double dtr = pi / 180.0;

            theta1 *= dtr;
            theta2 *= dtr;
            theta3 *= dtr;

            Double y1 = -(t + rf * Math.Cos(theta1));
            Double z1 = -rf * Math.Sin(theta1);

            Double y2 = (t + rf * Math.Cos(theta2)) * sin30;
            Double x2 = y2 * tan60;
            Double z2 = -rf * Math.Sin(theta2);

            Double y3 = (t + rf * Math.Cos(theta3)) * sin30;
            Double x3 = -y3 * tan60;
            Double z3 = -rf * Math.Sin(theta3);

            Double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

            Double w1 = y1 * y1 + z1 * z1;
            Double w2 = x2 * x2 + y2 * y2 + z2 * z2;
            Double w3 = x3 * x3 + y3 * y3 + z3 * z3;

            // x = (a1*z + b1)/dnm
            Double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
            Double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

            // y = (a2*z + b2)/dnm;
            Double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
            Double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

            // a*z^2 + b*z + c = 0
            Double a = a1 * a1 + a2 * a2 + dnm * dnm;
            Double b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
            Double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

            // discriminant
            Double d = b * b - (Double)4.0 * a * c;
            if (d < 0)
            {
                return -1; // non-existing point
            }

            z0 = -0.5 * (b + Math.Sqrt(d)) / a;
            x0 = (a1 * z0 + b1) / dnm;
            y0 = (a2 * z0 + b2) / dnm;
            return 0;
        }

        private void button3_Click(object sender, EventArgs e)

        {
            // inverse kinematics
            // helper functions, calculates angle theta1 (for YZ-pane)
            Double X1 = Convert.ToDouble(textBox2.Text);
            Double Y1 = Convert.ToDouble(textBox3.Text);
            Double Z1 = Convert.ToDouble(textBox4.Text);
            Double T1=0;
            Double T2=0;
            Double T3=0;
          

            int Status2 = delta_calcInverse(X1, Y1, Z1, ref T1, ref T2, ref T3);
            if (Status2 == 0)
            {
                textBox8.Text = "OK";
                textBox5.Text = Convert.ToString(T1);
                textBox6.Text = Convert.ToString(T2);
                textBox7.Text = Convert.ToString(T3);
            }
            else
            {
                textBox8.Text = "non-existing position";
                textBox5.Text = " Ø ";
                textBox6.Text = " Ø ";
                textBox7.Text = " Ø ";
            }




        }
        // inverse kinematics
        // helper functions, calculates angle theta1 (for YZ-pane)
        int delta_calcAngleYZ(Double x0, Double y0, Double z0,ref Double theta)
        {
            

            Double y1 = -0.5 * 0.57735 * ff; // f/2 * tg 30
            y0 -= 0.5 * 0.57735 * ee;    // shift center to edge
                                         // z = a + b*y
            Double a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
            Double b = (y1 - y0) / z0;
            // discriminant
            Double d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
            if (d < 0) return -1; // non-existing point
            Double yj = (y1 - a * b - Math.Sqrt(d)) / (b * b + 1); // choosing outer point
            Double zj = a + b * yj;
            theta = 180.0 * Math.Atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
            return 0;
        }

        // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
        // returned status: 0=OK, -1=non-existing position
        int delta_calcInverse(Double x0, Double y0, Double z0, ref Double theta1, ref Double theta2, ref Double theta3)
        {
            theta1 = theta2 = theta3 = 0;
            int status = delta_calcAngleYZ(x0, y0, z0, ref theta1);
            if (status == 0) status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0,ref theta2);  // rotate coords to +120 deg
            if (status == 0) status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0,ref theta3);  // rotate coords to -120 deg
            return status;
        }

        private void button6_Click(object sender, EventArgs e)
        {
            deltabot_Parameters cp = new deltabot1_interface.deltabot_Parameters(this);
            cp.StartPosition = FormStartPosition.CenterParent;
            cp.Show();
        }

        private void textBox8_TextChanged(object sender, EventArgs e)
        {

        }

        private void button5_Click(object sender, EventArgs e)
        {

           
            Double T12 = Convert.ToDouble(textBox5.Text);
            Double T22  = Convert.ToDouble(textBox6.Text);
            Double T32  = Convert.ToDouble(textBox7.Text);
            Double RT12 = Math.Round(T12);
            Double RT22 = Math.Round(T22);
            Double RT32 = Math.Round(T32);
            String st = ("X" + Convert.ToString(RT12) + "Y" + Convert.ToString(RT22) + "Z" + Convert.ToString(RT32));
            textBox8.Text = st;
            port.WriteLine(st);


        }

        private void textBox5_TextChanged(object sender, EventArgs e)
        {

        }

        private async void button7_Click(object sender, EventArgs e)
        {
            Pause = false;
            Fetch = false;
            Capture = new VideoCapture(Convert.ToInt32(numericUpDown1.Value));
            try
            {
                while (!Pause)
                {

                    Mat m = new Mat();
                    Capture.Read(m);
                    if (!m.IsEmpty)
                    {
                        pictureBox1.Image = m.Bitmap;
                        Image<Bgr, byte> img1 = new Image<Bgr, byte>(m.Bitmap);
                        Image<Gray, Byte> gray1 = img1.Convert<Gray, Byte>().PyrUp().PyrDown();

                        CircleF[] circles = gray1.HoughCircles(
                        new Gray(180),
                        new Gray(120),
                        2,
                        10,
                        0,
                        0)[0];
                        Image<Bgr, byte> imageCircles = img1.CopyBlank();
                        foreach (CircleF circle in circles)
                        {
                            imageCircles.Draw(circle, new Bgr(System.Drawing.Color.Red),2);
                            Double Xcircle = circle.Center.X;
                            Double Ycircle = circle.Center.Y;
                            Double Rcircle = circle.Radius;

                            int Xint = Convert.ToInt32(Xcircle);
                            int Yint = Convert.ToInt32(Ycircle);
                            
                            String Coo = ("X" + Convert.ToString(Xcircle) + ";Y=" + Convert.ToString(Ycircle)+"R="+ Convert.ToString(Rcircle));
                            int b = (int)imageCircles.Data[Xint,Yint, 0];
                            int g = (int)imageCircles.Data[Xint, Yint, 1];
                            int r = (int)imageCircles.Data[Xint, Yint, 2];
    
                            CvInvoke.PutText(imageCircles, Coo, new System.Drawing.Point(Xint, Yint+10), Emgu.CV.CvEnum.FontFace.HersheyPlain, 1.0, new Bgr(0, 255, 100).MCvScalar);


                            if (Fetch == true)
                            {
                                Double RR1 = Math.Round(Rcircle);
                                Double RX1 = Math.Round(Xcircle );
                                Double RY1 = Math.Round(Ycircle);
                                Double coef = 30 / RR1;
                                Double RX2 = (RX1-320)*coef;
                                Double RY2 = (RY1-240)*coef;
                                textBox8.Text = "OK";
                                textBox2.Text = Convert.ToString(RX2);
                                textBox3.Text = Convert.ToString(RY2);
                            }

                        }
                        Image<Bgr, byte> overlay = imageCircles.Copy();
                        CvInvoke.AddWeighted(overlay, 1, m, 1.2, 0, m);
                        pictureBox1.Image = m.Bitmap;
                        await Task.Delay(fps);
                    }
                    else
                    {
                        break;
                    }
                }

            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            

        }

        private void label8_Click(object sender, EventArgs e)
        {

        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }

        private void button8_Click(object sender, EventArgs e)
        {
            Pause = true;
        }

        private void button11_Click(object sender, EventArgs e)
        {

            port.WriteLine("M3");
        }

        private void button12_Click(object sender, EventArgs e)
        {
            port.WriteLine("M4");
        }

        private void button10_Click(object sender, EventArgs e)
        {
            port.WriteLine("X0Y0Z0");
        }

        private void button9_Click(object sender, EventArgs e)
        {
            Fetch = true;
        }

        private void textBox2_TextChanged(object sender, EventArgs e)
        {

        }
    }
}
