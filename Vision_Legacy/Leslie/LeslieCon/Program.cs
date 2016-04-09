using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;
using System.IO;
using System.IO.Ports;
using NGCP.IO;
using System.Management;
using LisaControlerCsh;

namespace LeslieCon
{
    public class Program
    {
        const string RegPath = @"SOFTWARE\NGCP\UGV\VISION";
        static Lidar lidar;
        static VisionHandler video;
        static UdpClientSocket socket;
        static bool debug = true;
        static bool useVideo = true;

        static void Main(string[] args)
        {
            var dllDirectory = Environment.CurrentDirectory + "\\openCV";
            Environment.SetEnvironmentVariable("PATH", Environment.GetEnvironmentVariable("PATH") + ";" + dllDirectory);
            try
            {
                //host IP, host Port, client IP, client Port; client is the behavior code, host is the vision code
                socket = new UdpClientSocket(System.Net.IPAddress.Parse("127.0.0.1"), 5550, System.Net.IPAddress.Parse("127.0.0.1"), 5500);
                socket.Start();

                bool found = false;
                Console.WriteLine("Initializing Lidar...");
                using (var searcher = new ManagementObjectSearcher
                ("SELECT * FROM WIN32_SerialPort"))
                {
                    string[] portnames = SerialPort.GetPortNames();
                    var ports = searcher.Get().Cast<ManagementBaseObject>().ToList();
                    foreach (var port in ports)
                    {
                        if ((port["Caption"] as string).Contains("URG Series USB Device Driver"))
                        {
                            string lidarPort = port["DeviceID"].ToString();
                            Console.WriteLine("Lidar on port: " + lidarPort);
                            lidar = new Lidar(lidarPort);
                            found = true;
                        }
                    }
                }
                if (!found)
                {
                    Console.WriteLine("LIDAR not found!");
                    Console.ReadLine();
                    return;
                }

                if (useVideo)//Initialize the video system if it is enabled
                {
                    Console.WriteLine("Initializing Camera...");
                    video = new VisionHandler(debug);
                }

            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                Console.WriteLine(ex.StackTrace);
                Console.ReadLine();
                return;
            }

            mainLoop();
        }

        static void mainLoop()//Main processing loop. Runs all enabled systems
        {
            while (true)
            {
                StringBuilder consoleOut = new StringBuilder();
                consoleOut.Append("UGV Vision console" + "\n");
                if (useVideo)//Run video logic
                {
                    video.RunFrame();
                    consoleOut.Append("Targets: 0" + "\n");
                    VisionHandler.Target printTar = new VisionHandler.Target();
                    consoleOut.Append(printTar.ToString() + "\n");
                }
                Console.Clear();
                Console.Write(consoleOut.ToString());
            }
        }
    }
}
