
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using LisaControlerCsh;
using LeahDLL;

namespace LeslieCon
{
    class VisionHandler
    {
        private Leah camera;
        private const int THRESHOLD = 100;
        private int TargetSize;
        private const int TARINC = 6;
        public struct Target
        {
            public Target(int dataX, int dataY, int dataR, float dataH)
            {
                CenterX = dataX;
                CenterY = dataY;
                Radius = dataR;
                Heading = dataH;
            }
            public int CenterX;
            public int CenterY;
            public int Radius;
            public float Heading;
            public override string ToString()
            {
                return "X: " + CenterX.ToString() + " Y: " + CenterY.ToString() + " R: " + Radius.ToString();
            }
        }

        public VisionHandler(bool debug = false)
        {
            camera = new Leah();
            camera.initVideo(0, 2048, 2048, THRESHOLD, debug);

        }

        public void RunFrame()
        {
            camera.process();
        }

        public List<Target> getTarget()
        {
            List<Target> output = null;
            int[] data = new int[TargetSize];
            bool good = false;
            if (camera.targetFound())
            {
                output = new List<Target>();
                while (!good)
                {
                    unsafe
                    {
                        fixed (int* ptr = data)
                        {
                            good = camera.getTarget(ptr, TargetSize);
                        }
                    }
                    if (!good)
                    {
                        TargetSize += TARINC;
                        data = new int[TargetSize];
                    }

                }
                for (int i = 0; i < TargetSize; i += 3)
                {
                    int X = data[i];
                    int Y = data[i+1];
                    float H = camera.mapHeading(X, Y);
                    output.Add(new Target(X, Y, data[i + 2], H));
                }
            }
            return output;
        }
    }
}
