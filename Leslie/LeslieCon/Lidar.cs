using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;


namespace LisaControlerCsh
{
    public class Lidar
    {
        private static SerialPort serialPort;

        private int byteCount;
        private int charCount;
        private byte[] value;
        private int dataCount;
        private int[] recievedData;
        private bool DataReady;
        private int dataTime;
        public bool dataBad;

        private CircularArray buffer;

        public Lidar(string COMport)
        {
            this.Connect(COMport);
        }

        private void Connect(string COMport)
        {
            if (serialPort == null)
            {
                serialPort = new SerialPort();
            }

            if (!serialPort.IsOpen)
            {
                    serialPort.PortName = COMport;
                    serialPort.BaudRate = 115200;
                    serialPort.ReadBufferSize = 10000;
                    serialPort.Parity = Parity.None;
                    serialPort.DataReceived += new SerialDataReceivedEventHandler(comPort_DataReceived);
                    serialPort.ReceivedBytesThreshold = 2;
                    serialPort.Open();
            }
        }

        public void Disconnect()
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }

        public bool Connected()
        {
            if (serialPort != null)
                return serialPort.IsOpen;
            else
                return false;
        }

        public void RequestData()
        {
            byte[] dataPacket = new byte[17];
            dataPacket[0] = 0x00;//command byte 1
            dataPacket[1] = 0x4d;//command byte 0
            dataPacket[2] = 0x44;//Prameter byte two character encoding
            dataPacket[3] = 0x30;
            dataPacket[4] = 0x30;
            dataPacket[5] = 0x30;
            dataPacket[6] = 0x30;
            dataPacket[7] = 0x31;
            dataPacket[8] = 0x30;
            dataPacket[9] = 0x38;
            dataPacket[10] = 0x30;
            dataPacket[11] = 0x30;//cluster count byte 1
            dataPacket[12] = 0x31;//cluster count byte 0
            dataPacket[13] = 0x30;//scna interval
            dataPacket[14] = 0x30;//Number of scans byte 1
            dataPacket[15] = 0x31;//Number of scans byte 0
            dataPacket[16] = (byte)'\n';//inset Line terminator
            serialPort.Write(dataPacket, 0, 17);//send request

            byteCount = 0;
            charCount = 0;
            dataCount = 0;
            dataBad = false;
            value =  new byte[3];
            recievedData = new int[1180];
            DataReady = false;
            buffer = new CircularArray(3);
            dataTime = 0;
            return;

        }

        void comPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                while (serialPort.BytesToRead > 0)
                {
                    for (int i = 0; i < serialPort.BytesToRead; i++)
                    {
                        
                        byte data = (byte)serialPort.ReadByte();

                        if (dataTime > 7)
                        {
                            if (data == 0x0a || (byteCount - 45) % 66 == 0)
                            {
                                //skip 0x0a characters
                            }
                            else if (charCount < 2)
                            {
                                value[charCount] = (byte)(data - 0x30);
                                charCount++;

                            }
                            else
                            {

                                value[charCount] = (byte)(data - 0x30);
                                recievedData[dataCount] = (int)((value[0] << 12) + (value[1] << 6) + value[2]);
                                charCount = 0;
                                value = new byte[3];
                                dataCount++;
                            }
                        }
                        else
                        {
                            buffer.push(data);
                            string bu = buffer.read();
                            if (bu == "393962")
                            {
                                dataTime = 1;
                                if (byteCount != 39)
                                    dataBad = true;
                            }
                            if (dataTime > 0)
                            {
                                dataTime++;
                            }
                        }
                        if (dataCount >= 1000)
                        {
                            DataReady = true;
                            
                        }
                        byteCount++;
                    }

                }
            }
            catch
            {

            }
        }

        public bool IsDataReady()
        {
            return DataReady;
        }

        public int[] GetData()
        {
            if (DataReady)
            {
                RequestData();
                return recievedData;
            }
            else return null;
        }
    }

    private class CircularArray
    {
        dataObj current;
        public void push(object newdata)
        {
            current.data = newdata;
            current = current.next;
        }
        public CircularArray(int Size)
        {
            current = new dataObj();
            current.tag = 0;
            current.data = new byte();
            dataObj head = current;

            for(int i = 1; i < Size; i++)
            {
                current.next = new dataObj();
                current = current.next;
                current.tag = i;
                current.data = new byte();
            }
            current.next = head;
        }
        public string read()
        {
            string output = "";
            dataObj start = current;
            do
            {
                output += ((byte)current.data).ToString("x2");
                current = current.next;
            } while (current.tag != start.tag);
            return output;
        }
    }
    private class dataObj
    {
        public dataObj next;
        public object data;
        public int tag;
    }
}
