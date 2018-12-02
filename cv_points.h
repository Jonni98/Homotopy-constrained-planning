#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
using namespace cv;
using namespace std;
vector<Point2f> find_points(int width, int height, double** map_array)
{
  Mat map(height, width, CV_8UC1,Scalar(0));
  for(int y=0;y<height;y++)
 {
    for(int x=0;x<width;x++)
    {
      uchar color = map.at<uchar>(Point(x,y));
      color = map_array[x][y];
      map.at<uchar>(Point(x,y)) = color;
    }
  }
  Mat grayscaleMap;
  Mat thr,thresh;
  threshold(map, thr, 100,255,THRESH_BINARY);
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(thr, contours,hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
  vector<Moments> mu(contours.size());
  for( int i = 0; i<contours.size(); i++ )
  {
    mu[i] = moments( contours[i], false );
  }

  // get the centroid of figures.
  vector<Point2f> mc(contours.size());
  for( int i = 0; i<contours.size(); i++)
  {
    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    // cout<<mc<<'\n';
  }
  // for(int i=0;i<mc.size();i++)
  // cout<<mc[i]<<'\n';
  // imshow("hi",thr);
  // waitKey(0);
  return mc;
}
