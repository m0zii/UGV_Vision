using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

//NOTE: all distances are in mm. All angles are in degrees.
namespace ObstacleDetection
{
    class ObstacleDetector
    {
        private List<double> stepDistances;
        private List<Obstacle> obstacles;
        private int startStep;

        //Objects that are farther than the MAX_DISTANCE
        //Will not be considered when forming an obstacle 
        private const int MAX_DISTANCE = 15000;

        //The maximum amount of lidar readings that can be used
        // to create an obstacle
        private const int RESOLUTION = 20;

        //The largest difference allowed between two successive
        //lidar readings 
        private const int THRESHOLD = 200;

        public ObstacleDetector(int startingStep, int maxDistance, int threshold,
                                int resolution, List<double> distanceList)
        {
            stepDistances = distanceList;
            startStep = startingStep;
            obstacles = new List<Obstacle>();
        }

        public ObstacleDetector(int startingStep,  List<double> distanceList)
        {
            stepDistances = distanceList;
            startStep = startingStep;
            obstacles = new List<Obstacle>();
        }

        /**
         Takes a list of distance vectors and creates
         a list of obstacles based on the distance vectors.  
        **/
        public List<Obstacle> findObstacles()
        {
            double startDistance;
            double endDistance;
            double shortestDistance;
            double shortestAngle;
            double startAngle;
            double endAngle;
            int step;
            int count;
            for (int i = 0; i < stepDistances.Count; i++)
            {
                count = 0;
                step = i + startStep;

                //Ignore Data that is greater than maxDistance
                if (stepDistances[i] > MAX_DISTANCE)
                {
                    continue;
                }

                //Check the data list bounds
                if (i + 1 >= stepDistances.Count)
                {
                    break;
                }

                //Set Start Distance and start angle of object
                startDistance = stepDistances[i];
                startAngle = stepToDegree(step);
                shortestDistance = startDistance;
                shortestAngle = startAngle;

                // Get Data points for object while next data point is not greater than 
                // a set threshold
                while (Math.Abs(stepDistances[i] - stepDistances[i + 1]) < THRESHOLD)
                {
                    i++;
                    step++;
                    count++;

                    //Check if the amount of datapoints in object is less than the resolution
                    if (count > RESOLUTION)
                    {
                        i--;
                        step--;
                        break;
                    }

                    //Check if current data point is greater than max distance
                    if (stepDistances[i] > MAX_DISTANCE)
                    {
                        break;
                    }

                    //Check if current data point is shortest distance. 
                    if (shortestDistance > stepDistances[i])
                    {
                        shortestDistance = stepDistances[i];
                        shortestAngle = stepToDegree(step);
                    }

                    //Check the data list bounds
                    if (i + 1 >= stepDistances.Count)
                    {
                        break;
                    }
                }
                //Set end distance and end angle
                endDistance = stepDistances[i];
                endAngle = stepToDegree(step);

                //Create new obstacle and add to obstacle list
                obstacles.Add(new Obstacle(startDistance, endDistance, shortestDistance,
                                           startAngle, endAngle, shortestAngle));
            }
            return obstacles;
        }

        /**
        Converts a step from the lidar into a degree.
        Note that each step on the the Hokuyo UTM-30LX lidar
        is separated by .25 degrees. The lidar has a 270 degree
        range of detection. Step 0 occurs at angle 315 degrees.  
        **/
        private double stepToDegree(int step)
        {
            double startAngle = 315;
            double angle = startAngle + step * .25f;
            if (angle >= 360)
            {
                angle = angle - 360;
            }
            return angle;
        }
    }
}
