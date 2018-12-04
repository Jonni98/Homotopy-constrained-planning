#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
// #include "get_representativepoints.h"
#define MAP_WIDTH 500
#define MAP_HEIGHT 500
#include "cv_points.h"
#include "signatures.h"
// #include "mha*.h"
#include "planner4.h"
void create_map(double** map2)
{
  for(int i=0;i<MAP_WIDTH;i++)
  {
    for(int j =0;j<MAP_HEIGHT;j++)
      map2[i][j] = 0;
  }

  for(int i=200;i<300;i++)
  {
    for(int j =200;j<300;j++)
      map2[i][j] = 255;
  }

  for(int i=100;i<150;i++)
  {
    for(int j =100;j<150;j++)
      map2[i][j] = 255;
  }
  for(int i=400;i<450;i++)
  {
    for(int j =100;j<150;j++)
      map2[i][j] = 255;
  }
}

int main()
{
  double **map2 = new double*[MAP_WIDTH];
  for(int i=0;i<MAP_WIDTH;i++)
  {
    map2[i] = new double[MAP_HEIGHT];
  }
  create_map(map2);
  vector<Point2f> representative_points= find_points(MAP_WIDTH,MAP_HEIGHT, map2);
  generate_signatures(representative_points);
  string desired_signature = "2";
  planner(map2,desired_signature,representative_points,MAP_WIDTH, MAP_HEIGHT);
  return 1;
  // for(int )
}
