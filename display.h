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

void colored_display_map(double ** map, int height, int width)
{
  // cout<<"HI";
  Mat cv_map(height, width, CV_8UC3,Scalar(0));
  for(int y=0;y<height;y++)
  {
    for(int x=0;x<width;x++)
    {
      Vec3b color = cv_map.at<Vec3b>(Point(x,y));
      if(map[x][y]==0)
      {
          color[0] = 255;
          color[1] = 255;
          color[2] = 255;
          // cout << "Pixel >200 :" << x << "," << y << endl;
      }
      else if(map[x][y]==255)
      {
        // cout<<"HI";
          color[0] = 0;
          color[1] = 255;
          color[2] = 0;
      }
      else
      {
        // cout<<"WHATU da fuck"<<'\n';
        color[0] = 255;
        color[1] = 0;
        color[2] = 0;
      }
      cv_map.at<Vec3b>(Point(x,y)) = color;
    }
  }
  imshow("hi",cv_map);
  waitKey(0);
}

void colored_display_path(double ** map, int height, int width, vector<node*> path)
{
  // cout<<"HI";
  waitKey(2000);
  Mat cv_map(height, width, CV_8UC3,Scalar(0));
  for(int y=0;y<height;y++)
  {
    for(int x=0;x<width;x++)
    {
      Vec3b color = cv_map.at<Vec3b>(Point(x,y));
      if(map[x][y]!=255)
      {
          color[0] = 255;
          color[1] = 255;
          color[2] = 255;
          // cout << "Pixel >200 :" << x << "," << y << endl;
      }
      else
      {
        // cout<<"HI";
          color[0] = 0;
          color[1] = 255;
          color[2] = 0;
      }
      cv_map.at<Vec3b>(Point(x,y)) = color;
    }
  }
  Vec3b color1 = cv_map.at<Vec3b>(Point(path[0]->x,path[0]->y));
  color1[0] = 0;
  color1[1] = 0;
  color1[2] = 255;
  cv_map.at<Vec3b>(Point(path[0]->x,path[0]->y)) = color1;
  imshow("hi",cv_map);
  waitKey(0);
  for(int i=path.size()-1;i>=0;i--)
  {
    Vec3b color1 = cv_map.at<Vec3b>(Point(path[i]->x,path[i]->y));
    color1[0] = 255;
    color1[1] = 0;
    color1[2] = 0;
    cv_map.at<Vec3b>(Point(path[i]->x,path[i]->y)) = color1;
    imshow("hi",cv_map);
    waitKey(20);
  }
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
