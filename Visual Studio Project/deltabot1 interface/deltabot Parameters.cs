using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace deltabot1_interface
{
    public partial class deltabot_Parameters : Form
    {
        Form1 _Form1;
       
        public deltabot_Parameters()
        {
            InitializeComponent();
        }

        public deltabot_Parameters(Form1 fr)
        {
            InitializeComponent();
            _Form1 = fr;
            


        }

        private void button2_Click(object sender, EventArgs e)
        {   
            
            this.Close();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (_Form1 != null)
            {
               Double e2 = Convert.ToDouble(textBox1.Text);
               Double f2 = Convert.ToDouble(textBox2.Text);
               Double re2 = Convert.ToDouble(textBox3.Text);
               Double rf2 = Convert.ToDouble(textBox4.Text);

                _Form1.Parameters(e2,f2,re2,rf2);
            }
        }
    }
}
