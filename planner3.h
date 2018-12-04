#include<queue>
#include<math.h>
#include <unordered_map>
#define NUMOFDIRS 8
int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
// #define NUMOFDIRS 4
// int dX[NUMOFDIRS] = {-1, 1, 0,  0};
// int dY[NUMOFDIRS] = {0,  0,  -1, 1};
class node
{
public:
  int x,y;
  string signature;
  float g,h,f;
  node* parent;
  node()
  {
    x = y = 0;
    g = h = f = 10000;
    signature = "";
  }
};
struct Compare {
    bool operator()(node *n1, node *n2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return n1->f > n2->f;
    }
};
float heuristic(node* current,node* goal)
{
  return sqrt(pow(current->x-goal->x,2)+pow(current->y-goal->y,2));
}

#include "display.h"
void print_path(double**map,node* goal, node *start, int y_size, int x_size)
{
  node*current = goal;
  while(current->x!=start->x&&current->y!=start->y)
  {
    cout<<current->x<<' '<<current->y<<' '<<current->signature<<'\n';
    map[current->x][current->y] = 122;
    current = current->parent;
  }
  display_map(map,y_size,x_size);
}
#include "append&check_signatures.h"
int planner(double** map,string desired_signature, vector<Point2f> representative_points, int map_x_size, int map_y_size)
{
  double open[500][500];
  for(int i=0;i<500;i++)
  {
    for(int j=0;j<500;j++)
      open[i][j] = 0;
  }
  std::priority_queue<node*, std::vector<node*>, Compare> open;
  int collision_threshold = 255;
  cout<<representative_points[0]<<'\n';
  cout<<representative_points[1]<<'\n';
  cout<<representative_points[2]<<'\n';
  string state_signature = "";
  node* start; node* goal;
  goal = new node;
  goal->x = 474;goal->y = 350; goal->h = 0;
  goal->signature = "";
  start = new node;
  start->x = 0;
  start->y = 0;
  start->g = 0;
  start->h = heuristic(start,goal);
  open.push(start);
  
