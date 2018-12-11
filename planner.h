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
  while(current->x!=start->x&&current->y!=start->y)
  {
    cout<<current->x<<' '<<current->y<<' '<<current->signature<<'\n';
    map[current->x][current->y] = 122;
    current = current->parent;
  }
  display_map(map,y_size,x_size);
}
#include "append&check_signatures.h"
float my_heuristic(node* current, node* goal)
{
  int heuristic = 0;
  int a[100]; int k=0;
  vector<int> b,c;
  for(int j=0;j<current->signature.size();j++)
  {
    if(current->signature[j]=='-')
    {
      j++;
      b.push_back(-(int(current->signature[j])-'0'));
    }
    else b.push_back((int(current->signature[j])-'0'));
  }
  for(int j=0;j<goal->signature.size();j++)
  {
    if(goal->signature[j]=='-')
    {
      j++;
      c.push_back(-(int(goal->signature[j])-'0'));
    }// k++;
    else c.push_back((int(goal->signature[j])-'0'));
  }
  if(b.size()==0) return c.size();
  else if(c.size()==0) return b.size();
  // cout<<b.size()<<' '<<c.size()<<'\n';
  if(b.size()>c.size())
  {
    for(int k=0;k<c.size();k++)
    {
      if(b[k]!=c[k]) heuristic+=(c.size()-k);
    }
    heuristic+=b.size()-c.size();
  }
  else
  {
    // cout<<"hi";
    for(int k=0;k<b.size();k++)
    {
      // cout<<b[k]<<' '<<c[k]<<'\n';
      if(b[k]!=c[k])
      {
        // cout<<"HI"<<'\n';
        heuristic+=(b.size()-k);
      }
    }
    heuristic+=c.size()-b.size();
  }
  return heuristic;
}

int planner(double** map,string desired_signature, vector<Point2f> representative_points, int map_x_size, int map_y_size)
{
  std::priority_queue<node*, std::vector<node*>, Compare> open;
  int collision_threshold = 255;
  cout<<representative_points[0]<<'\n';
  cout<<representative_points[1]<<'\n';
  cout<<representative_points[2]<<'\n';
  string state_signature = "";
  std::unordered_map <std::string, int> open_map;
  std::unordered_map <std::string, int> closed_map;
  std::unordered_map <std::string, node*> open_map_of_states;
  std::unordered_map <std::string, node*> closed_map_of_states;
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
  open_map[Hash(start)] = 1;
  open_map_of_states[Hash(start)] = start;
  int number_of_expansions = 0;
  // cout<<"Signature 1 "<<Hash(start)<<'\n';
  start->signature = "";
  // cout<<my_heuristic(start,goal);
  bool b;
  int sum = 0;
  if(start->signature==goal->signature) b=true;
  else b = false;
  cout<<"Signatures "<<b<<'\n';
  while(!open.empty())
  {
    node* current;
    current = open.top();
    // if(current->x >= 400&&current->y<=100)//&&current->signature.size()==0)
    //   cout<<current->x<<' '<<current->y<<' '<<current->f<<'\n';
    number_of_expansions+=1;
    open.pop();
    closed_map[Hash(current)] = 1;
    closed_map_of_states[Hash(current)] = current;
    if(current->x==goal->x&&current->y==goal->y&&current->signature==goal->signature)
    {
      cout<<"goal Reached"<<'\n';
      cout<<"number_of_expansions "<<number_of_expansions<<'\n';
      print_path(map,current,start,map_y_size,map_x_size);
      return 0;
    }
    for(int i=0;i<NUMOFDIRS;i++)
    {
      node* successor;
      successor = new node;
      successor->x = current->x + dX[i];
      successor->y = current->y + dY[i];
      successor->parent = current;

      if(successor->x>0 && successor->y>0 && successor->x<map_x_size && successor->y<map_y_size)
      {
        if(map[successor->x][successor->y]<collision_threshold)
        {
          successor->g = current->g + heuristic(successor,current);
          string Signature_action = find_signature_action(current,successor,representative_points,dX[i]);
          successor->signature = new_append_signature(current->signature,Signature_action);
          successor->h = heuristic(successor,goal)+ 20000*my_heuristic(successor,goal);
          successor->f = successor->g+successor->h;

          std::unordered_map<std::string,int>::const_iterator got = closed_map.find(Hash(successor));
          if(got==closed_map.end())
          {
            std::unordered_map<std::string,int>::const_iterator got2 = open_map.find(Hash(successor));
            if(got2==open_map.end())
            {
              // if(successor->x==401 && current->signature.size()==0)
              // cout<<"IT is"<<current->x<<' '<<successor->y<<' '<<successor->f<<'\n';
              open.push(successor);
              open_map[Hash(successor)] = 1;
              open_map_of_states[Hash(successor)] = successor;
            }
            else if(successor->f < open_map_of_states[Hash(successor)]->f)
            {
              // if(successor->x==401 && current->signature.size()==0)
              // cout<<"IT is"<<current->x<<' '<<successor->y<<' '<<successor->f<<'\n';
              // cout<<"replaced"<<'\n';
              open_map_of_states[Hash(successor)] = successor;
              open.push(successor);
            }
          }
        }
      }
    }
  }

}
