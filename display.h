void display_map(double ** map, int height, int width)
{
  // cout<<"HI";
  Mat cv_map(height, width, CV_8UC1,Scalar(0));
  // cout<<"HIW";
  for(int y=0;y<height;y++)
 {
    for(int x=0;x<width;x++)
    {
      uchar color = cv_map.at<uchar>(Point(x,y));
      color = map[x][y];
      cv_map.at<uchar>(Point(x,y)) = color;
    }
  }
  // cout<<"WTf";
  imshow("hi",cv_map);
  waitKey(0);
}

void display_expanded_states(double ** map, int height, int width, std::vector<Point> data)
{
  Mat cv_map(height, width, CV_8UC1,Scalar(0));
  for(int y=0;y<height;y++)
 {
    for(int x=0;x<width;x++)
    {
      uchar color = cv_map.at<uchar>(Point(x,y));
      color = map[x][y];
      cv_map.at<uchar>(Point(x,y)) = color;
    }
  }
  for(int k=0;k<data.size();k++)
  {
    uchar color = cv_map.at<uchar>(data[k]);
    color = 122;
    cv_map.at<uchar>(data[k]) = color;
  }

  // cout<<"WTf";
  imshow("hi",cv_map);
  waitKey(0);
}
