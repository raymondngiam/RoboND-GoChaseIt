#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global variable to keep track of last set robot speed
float robotSpeed = 0;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service DriveToTarget.");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  const uint8_t WHITE_PIXEL = 255;
  const float LINEAR_X = 0.1;
  const float ANGULAR_Z = 0.1;

  // TODO: Loop through each pixel in the image and check if there's a bright white one
  // Then, identify if this pixel falls in the left, mid, or right side of the image
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera

  //ROS_INFO("Image height: [%d]", img.height);  //returns 800
  //ROS_INFO("Image width: [%d]", img.width);    //returns 800
  //ROS_INFO("Image step: [%d]", img.step);      //returns 2400
  const uint8_t CHANNELS = 3;
  uint32_t oneThirdWidth = img.width / 3;
  bool whiteBlobFound = false;

  for (uint32_t r=0; r<img.height; r++){
    for (uint32_t c=0; c<img.width; c++){
      uint32_t index = (r*img.step) + c*CHANNELS;  //img.data is a flatten 1D array of
      uint8_t grayValueR = img.data[index];        //inital 2D array of shape [img.height,img.width*CHANNELS]
      uint8_t grayValueG = img.data[index+1];     
      uint8_t grayValueB = img.data[index+2];

      if (grayValueR == WHITE_PIXEL && \
      grayValueG == WHITE_PIXEL && \
      grayValueB == WHITE_PIXEL)
      {
        ROS_INFO("White blob detected at row[%d], column[%d].", r, c);
        whiteBlobFound = true;
        
        robotSpeed = LINEAR_X;
        int positionBin = c / oneThirdWidth;
        switch (positionBin)
        {
          case 0:
            ROS_INFO("White blob on left. Steering left.");
            drive_robot(0.0, 1.0*ANGULAR_Z);
            break;
          case 1:
            ROS_INFO("White blob on middle. Driving straight.");
            drive_robot(robotSpeed, 0.0);
          break;
          case 2:
            ROS_INFO("White blob on right. Steering right.");
            drive_robot(0.0, -1.0*ANGULAR_Z);
          break;
          default:
            break;
        }        
        break; //break out of the for loop once first white pixel is detected.
      }
    }
    if (whiteBlobFound)
    {
      break; //break out of the for loop once first white pixel is detected.
    }
  }
  if (!whiteBlobFound){ //Request a stop when there's no white ball seen by the camera
    if (robotSpeed != 0){  //Keep track of global variable, send service call to stop robot only once.
      robotSpeed = 0;
      drive_robot(robotSpeed, 0.0);
    }
  }
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}