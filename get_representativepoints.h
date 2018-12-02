#include<unordered_map>
#include<vector>
#include<queue>
#define NUMOFDIRS 8
int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
class Point
{
public:
  int x,y;
  Point(int X, int Y)
  {
    x = X;
    y = Y;
  }
  Point()
  {
    x = 0;
    y = 0;
  }
  bool operator==(const Point& rhs)
  {
    if(this->x==rhs.x&&this->y==rhs.y)
      return true;
    return false;
  }
};
std::string Hash(Point pt)
{
  std::string string;
  string = std::to_string(pt.x);
  string.append(std::to_string(pt.y));
  return string;
}
bool check_if_connected(Point pt, std::vector<Point> points, int collision_thresh, double** map, int X_size,
  int Y_size)
{

  for(int k=0;k<points.size();k++)
  {
    std::unordered_map <std::string, int> open_map;
    std::unordered_map <std::string, int> closed_map;

    Point goal= points[k];
    Point start = pt;
    Point current;
    std::queue<Point> open;
    open.push(start);
    open_map[Hash(start)]=1;
    while(!open.empty())
    {
      current = open.front();
      open.pop();
      closed_map[Hash(current)]=1;
      if(current==goal)
      {
        return true;
      }
      for(int succ_num=0;succ_num<8;succ_num++)
      {
        Point successor(current.x+dX[succ_num],current.y+dY[succ_num]);
        // if((int)map[successor.x,successor.y]>=collision_thresh && closed_map[Hash(successor)!=1]
        // && open_map[Hash(successor)!=1])
        if((int)map[(int)successor.x,(int)successor.y]>=collision_thresh)
        {
      //     open.push(successor);
      //     open_map[Hash(successor)]=1;
        }
      }
     }
  }
  return false;
}
// void get_representative_points(double **map, int collision_thresh, int X_size, int Y_size)
// {
//   std::vector<Point> points;
//   for(int i=0;i<X_size;i++)
//   {
//     for(int j=0;j<Y_size;j++)
//     {
//       if((int)map[i][j] >= collision_thresh)
//       {
//         Point p(i,j);
//           if(!check_if_connected(p,points,collision_thresh,map))
//           {
//             points.push_back(Point(i,j));
//           }
//       }
//     }
//   }
//   return points;
// }
