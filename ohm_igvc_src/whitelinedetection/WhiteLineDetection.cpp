#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "opencv2/videoio.hpp"
#include "flycapture/FlyCapture2.h"

class CameraProcessing
{
  private:
    FlyCapture2::Camera chameleon3; //The camera object for the Chameleon3 CM3-U3-31S4C-CS.
    FlyCapture2::Image rawImage, encodedImage; //Contains the raw and converted frames from the Chameleon3.
    cv::UMat convertedImage, hsvImage, binaryImage; //UMat containers for various frame transformations within the OpenCV format.

    short low_H, low_S, low_V; // lower limit for HSV slider
    short high_H, high_S, high_V; // upper limit for HSV slider


  public:
    /* Prototypes */
    void displayFrames()
    {
      chameleon3.Connect(0);
      chameleon3.StartCapture();

      while(true)
      {
          //Retrieves the image from the camera's buffer.
          chameleon3.RetrieveBuffer( &rawImage );

          //Converts rawImage to an image with a colorspace form.
          rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &encodedImage );

          //Converts encodedImage to OpenCV UMat.
          unsigned int rowBytes = (double)encodedImage.GetReceivedDataSize()/(double)encodedImage.GetRows();
          cv::Mat tempImage = cv::Mat(encodedImage.GetRows(), encodedImage.GetCols(), CV_8UC3, encodedImage.GetData(),rowBytes);
          convertedImage = tempImage.getUMat(cv::ACCESS_FAST);
          imageFiltering();

          cv::imshow("image", binaryImage);
          if(cv::waitKey(1) >= 0) break;
      }

      chameleon3.StopCapture();
      chameleon3.Disconnect();
    }

    CameraProcessing()
    {
      ros::NodeHandle nh;

      low_H = 30; low_S = 50; low_V = 60;
      high_H = 179; high_S = 255; high_V = 255;

      displayFrames();
    }

    void imageFiltering() //To be of type cv::UMat
    {
      cv::cvtColor(convertedImage, hsvImage, cv::COLOR_RGB2GRAY);
      cv::inRange(hsvImage, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), binaryImage);
      cv::erode(binaryImage, binaryImage, cv::UMat(), cv::Point(-1, -1), 5);
    }

    void warp(cv::UMat &convertedMatrix)  //To be of type cv::UMat
    {

    }

    void pointCloudDetection()
    {
      //Use opencv function
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "WhiteLineDetection");
  CameraProcessing cameraNode;

  return 0;
}
