using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Net.Sockets; 


namespace ObstacleDetection
{
    class Program
    {
        static void Main(string[] args)
        {
            const int GET_NUM = 1080;
            const int start_step = 0;
            const int end_step = 1080;

            //UDP Information 
            const string ip_address = "127.0.0.2";
            const string receiver_ip = "127.0.0.1";
            const int udp_port = 8008;


            //Serial Information 
            const int baudrate = 115200;
            const string port_name = "COM6";

            try
            {
                //Open Serial Port
                SerialPort urg = new SerialPort(port_name, baudrate);
                urg.NewLine = "\n\n";
                urg.Open();

                //Open UDp port
                UdpClient udp = new UdpClient(ip_address, udp_port);
                udp.Connect(receiver_ip, udp_port);

                //Set Lidar to Measurement Mode
                urg.Write(SCIP_Writer.setSCIP2Mode());
                urg.ReadLine(); //ignore SCIP echo back

                while(true)
                {
                    string test;
                    urg.Write(SCIP_Writer.requestSensorData(start_step, end_step));
                    test = urg.ReadLine(); // ignore echo back

                    //Receive data from the lidar
                    List<double> distances = new List<double>();
                    long time_stamp = 0;
                    for (int i = 0; i < GET_NUM; ++i)
                    {
                        string receive_data = urg.ReadLine();
                        if (!SCIP_Reader.getSensorData(receive_data, ref time_stamp, ref distances))
                        {
                            Console.WriteLine(receive_data);
                            break;
                        }
                        if (distances.Count == 0)
                        {
                            Console.WriteLine(receive_data);
                            continue;
                        }
                    }

                    //Parse Objects from lidar data
                    ObstacleDetector od = new ObstacleDetector(0, distances);
                    List<Obstacle> obstacles = od.findObstacles();

                    //Send objects to behavior
                    int num_obstacles = obstacles.Count;
                    int msgSize = (num_obstacles * 6)*sizeof(double) + sizeof(int);

                    byte[] obstacle_byte = obstacleToBytes(ref obstacles);
                    byte[] num_obstacles_byte = BitConverter.GetBytes(num_obstacles);
                    byte[] sendBytes = new byte[msgSize];

                    System.Buffer.BlockCopy(num_obstacles_byte, 0, sendBytes, 0, num_obstacles_byte.Length);
                    System.Buffer.BlockCopy(obstacle_byte, 0, sendBytes, num_obstacles_byte.Length, obstacle_byte.Length);

                    udp.Send(sendBytes, sendBytes.Length);
                   
                }
     

                urg.Write(SCIP_Writer.turnOffLaser()); // stop measurement mode
                urg.ReadLine(); // ignore echo back

                urg.Close();
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            finally
            {
                Console.WriteLine("Press any key.");
                Console.ReadKey();
            }
        }

        private static byte[] obstacleToBytes(ref List<Obstacle> obstacles)
        {
            int byteSize = (obstacles.Count * 6) * sizeof(double);
            byte[] obstacle_bytes = new byte[byteSize];
            int offset = 0;

            foreach (Obstacle obstacle in obstacles)
            {
                byte[] ob = new byte[(sizeof(double)*6)];

                List<byte[]> array = new List<byte[]>();
                array.Add(BitConverter.GetBytes(obstacle.getStartDistance()));
                array.Add(BitConverter.GetBytes(obstacle.getEndDistance()));
                array.Add(BitConverter.GetBytes(obstacle.getShortestDistance()));
                array.Add(BitConverter.GetBytes(obstacle.getStartAngle()));
                array.Add(BitConverter.GetBytes(obstacle.getEndAngle()));
                array.Add(BitConverter.GetBytes(obstacle.getShortestAngle()));

                int param_offset = 0;
                foreach (byte[] param in array)
                {
                    System.Buffer.BlockCopy(param, 0, ob, param_offset, param.Length);
                    param_offset += param.Length;
                }

                System.Buffer.BlockCopy(ob, 0, obstacle_bytes, offset, ob.Length);
                offset += ob.Length;
                ob = null;
            }

            return obstacle_bytes;
        }
    }
}