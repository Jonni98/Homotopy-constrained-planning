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
    // g = h = f = 10000;
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
float cost(node* current,node* goal)
{
  return sqrt(pow(current->x-goal->x,2)+pow(current->y-goal->y,2));
}

#include <boost/functional/hash.hpp>
struct State_Hasher
{
    size_t operator()(const node state1) const
    {
      using boost::hash_value;
      using boost::hash_combine;
        size_t seed = 0;
        seed+=hash_value(std::to_string(state1.x));
        seed+=hash_value(std::to_string(state1.y));
        seed+=hash_value(state1.signature);
        // for(GroundedCondition a:state1.conditions)
        // {
        //   seed+=hash_value(a.toString());
        // }
        // cout<<"HASH "<<seed<<'\n';
        return seed;
    }
};
std::string Hash(node* state)
{
  std::string string;
  string = std::to_string(state->x);
  string.append(std::to_string(state->y));
  string.append(state->signature);
  return string;
}
#include "display.h"
void print_path(double**map,node* goal, node *start, int y_size, int x_size)
{
  node*current = goal;
  while(current->x!=start->x||current->y!=start->y)
  {
    // cout<<current->x<<' '<<current->y<<' '<<current->signature<<'\n';
    map[current->x][current->y] = 122;
    current = current->parent;
  }
  display_map(map,y_size,x_size);
}
#include "append&check_signatures.h"


int planner(double** map,string desired_signature, vector<Point2f> representative_points, int map_x_size, int map_y_size)
{
  std::priority_queue<node*, std::vector<node*>, Compare> open;
  int collision_threshold = 255;
  cout<<representative_points[0]<<'\n';
  cout<<representative_points[1]<<'\n';
  cout<<representative_points[2]<<'\n';
  // string state_signature = "";
  // state_signature = append_signature(state_signature,"-B");
  // unordered_map<node,int,State_Hasher> open_map;
  std::unordered_map <std::string, int> open_map;
  std::unordered_map <std::string, int> closed_map;
  std::unordered_map <std::string, node*> open_map_of_states;
  std::unordered_map <std::string, node*> closed_map_of_states;
  // cout<<"Signature "<<'\n';
  // cout<<state_signature<<'\n';
  node* start; node* goal;
  goal = new node;
  goal->x = 499;goal->y = 499; goal->h = 0;
  goal->signature = "1";
  start = new node;
  start->x = 0;
  start->y = 0;
  start->g = 0;
  start->h = heuristic(start,goal);
  start->f = start->g + start->h;
  start->signature = "";
  open.push(start);
  open_map[Hash(start)] = 1;
  open_map_of_states[Hash(start)] = start;
  // cout<<map_x_size;
  while(!open.empty())
  {
    node* current;
    current = open.top();
    open.pop();
    closed_map[Hash(current)] = 1;
    closed_map_of_states[Hash(current)] = current;
    if(current->x==goal->x && current->y==goal->y && current->signature==goal->signature)
    {
      cout<<"goal Reached "<<current->g<<'\n';
      print_path(map,current,start,map_y_size,map_x_size);
      return 0;
    }
    if(current->x==105 &&current->y==20)
    cout<<"COST "<<current->g;
    for(int i=0;i<NUMOFDIRS;i++)
    {
      node* successor;
      successor = new node;
      successor->x = current->x + dX[i];
      successor->y = current->y + dY[i];
      successor->parent = current;
      if(successor->x>0 && successor->x<map_x_size && successor->y>0 && successor->y<map_y_size)
      {
        if(current->x==105&&current->y==20)
        cout<<successor->x<<' '<<successor->y<<'\n';
        // cout<<map[106][20];
        if(map[successor->x][successor->y]<collision_threshold)
        {
          successor->g = current->g + cost(successor,current);
          successor->h = heuristic(successor,goal);
          successor->f = successor->g+successor->h;
          string Signature_action = find_signature_action(current,successor,representative_points,dX[i]);
          successor->signature = append_signature(current->signature,Signature_action);
          if(successor->x==106 &&successor->y==20)
          cout<<"COST "<<successor->g<<' '<<current->x<<' '<<current->y<<'\n';
          if(closed_map[Hash(successor)]!= 1)
          {
            if(open_map[Hash(successor)]!=1)
            {
              open.push(successor);
              open_map[Hash(successor)] = 1;
              open_map_of_states[Hash(successor)] = successor;
            }
            else if(successor->f < open_map_of_states[Hash(successor)]->f)
            {
              open_map_of_states[Hash(successor)] = successor;
              open.push(successor);
            }
          }
        }
      }
    }
  }
}
