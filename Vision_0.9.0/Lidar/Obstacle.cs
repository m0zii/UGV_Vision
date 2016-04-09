using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ObstacleDetection
{
    /**
    Represents an Obstacle  
    **/
    class Obstacle
    {
        private double startDistance;
        private double endDistance;
        private double shortestDistance;
        private double startAngle;
        private double endAngle;
        private double shortestAngle;

        public Obstacle(double startDistance, double endDistance, double shortestDistance,
                        double startAngle, double endAngle, double shortestAngle)
        {
            this.startDistance = startDistance;
            this.endDistance = endDistance;
            this.shortestDistance = shortestDistance;
            this.startAngle = startAngle;
            this.endAngle = endAngle;
            this.shortestAngle = shortestAngle;
        }

        public double getStartDistance()
        {
            return startDistance;
        }

        public double getEndDistance()
        {
            return endDistance;
        }

        public double getShortestDistance()
        {
            return shortestDistance;
        }

        public double getStartAngle()
        {
            return startAngle;
        }

        public double getEndAngle()
        {
            return endAngle;
        }

        public double getShortestAngle()
        {
            return shortestAngle;
        }

        public void toString()
        {
            Console.WriteLine("Start Distance: " + startDistance);
            Console.WriteLine("End Distance: " + endDistance);
            Console.WriteLine("Shortest Distance: " + shortestDistance);
            Console.WriteLine("Start Angle: " + startAngle);
            Console.WriteLine("End Angle: " + endAngle);
            Console.WriteLine("Shortest Angle: " + shortestAngle);
        }
    }
}
