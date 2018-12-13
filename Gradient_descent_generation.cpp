#include <iostream>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include "cv_points.h"
#include "signatures.h"
// #include "mha*.h"
#include "planner4.h"
#define MAP_WIDTH 101
#define MAP_HEIGHT 101
void Gradient_Descent_create_map(double** map2)
{
  for(int i=0;i<MAP_WIDTH;i++)
  {
    for(int j =0;j<MAP_HEIGHT;j++)
      {
        if(map2[i][j]>=2) map2[i][j] =255;
        else map2[i][j] = 0;
        // map2[i][j] = 0;
      }
  }
  // imshow("hi",map2);
  // waitKey(map2);
}
int main(int argc, char * argv[])
{
    std::fstream myfile("/home/suhail/PlanningCourse/Project/cost_func.txt", std::ios_base::in);

    double a;
    double b;
    // while (myfile >> a)
    // {
    //     std::cout<<a<<'\n';
    //     b+=1;
    // }
    // std::cout<<b;
    double **map2 = new double*[MAP_WIDTH];
    for(int i=0;i<MAP_WIDTH;i++)
    {
      map2[i] = new double[MAP_HEIGHT];
    }
    for(int i=0;i<MAP_WIDTH;i++)
    {
      for(int k=0;k<MAP_HEIGHT;k++)
      {
        myfile>>a;
        map2[i][k] = a;
        // std::cout<<map2[i][k]<<'\n';
      }
    }
    Gradient_Descent_create_map(map2);
    vector<Point2f> representative_points= find_points(MAP_WIDTH,MAP_HEIGHT, map2);
    generate_signatures(representative_points);
    string desired_signature = "2";
    planner(map2,desired_signature,representative_points,MAP_WIDTH, MAP_HEIGHT);
    return 1;
    // getchar();

    return 0;
}
