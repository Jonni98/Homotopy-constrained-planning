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
  float g;
  int h[2],f[2];
  node* parent;
  node()
  {
    x = y = 0;
    g = 10000;
    signature = "";
  }
};
struct Compare {
    bool operator()(node *n1, node *n2) {
        // return "true" if "p1" is ordered before "p2", for example:
        return n1->f[0] > n2->f[0];
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
  std::priority_queue<node*, std::vector<node*>, Compare> ad_open;
  std::priority_queue<node*, std::vector<node*>, Compare> inad_open;

  int collision_threshold = 255;
  cout<<representative_points[0]<<'\n';
  cout<<representative_points[1]<<'\n';
  cout<<representative_points[2]<<'\n';
  string state_signature = "";
  std::unordered_map <std::string, int> inad_open_map;
  std::unordered_map <std::string, int> ad_open_map;
  std::unordered_map <std::string, int> inad_closed_map;
  std::unordered_map <std::string, int> ad_closed_map;

  std::unordered_map <std::string, node*> inad_open_map_of_states;
  std::unordered_map <std::string, node*> ad_open_map_of_states;
  std::unordered_map <std::string, node*> closed_map_of_states;
  // cout<<"Signature "<<'\n';
  // cout<<state_signature<<'\n';
  node* start; node* goal;
  goal = new node;
  goal->x = 474;goal->y = 350; goal->h[0] = goal->h[1] = 0;
  goal->signature = "";
  start = new node;
  start->x = 0;
  start->y = 0;
  start->g = 0;
  start->h[0] = (heuristic(start,goal));
  start->h[1] = (my_heuristic(start,goal));
  start->f[0] = start->h[0] + start->g;
  start->f[1] = start->h[1] + start->g;
  node* a;
  ad_open.push(start);
  a = ad_open.top();
  inad_open.push(start);
  ad_open_map[Hash(start)] = 1;
  inad_open_map[Hash(start)] = 1;
  ad_open_map_of_states[Hash(start)] = start;
  inad_open_map_of_states[Hash(start)] = start;
  int number_of_expansions = 0;
  float w2;
  // // cout<<"Signature 1 "<<Hash(start)<<'\n';
  start->signature = "";
  // bool b;
  // if(start->signature==goal->signature) b=true;
  // else b = false;
  // cout<<"Signatures "<<b<<'\n';


//  -----------------------
  while(!ad_open.empty()||!inad_open.empty())
  {
    node* ad; node* inad;
    ad = ad_open.top();
    inad = inad_open.top();
    if(inad->f[1]<=w2*ad->f[0])
    {
  //   number_of_expansions+=1;
      node* current;
      current = inad_open.top();
      inad_open.pop();
      inad_closed_map[Hash(current)] = 1;
  //   closed_map_of_states[Hash(current)] = current;
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
        // if(current->x>=420 && current->signature=="")
        // cout<<successor->x<<' '<<successor->y<<' '<<successor->signature<<'\n';
        successor->parent = current;
        if(successor->x>0&&successor->x<map_x_size&&successor->y>0&&successor->y<map_y_size)
        {
          // if(current->x>=420 && current->y<=100 && current->signature=="")
          // cout<<successor->x<<' '<<successor->y<<' '<<successor->signature<<'\n';
          if(map[successor->x][successor->y]<collision_threshold)
          {
            successor->g = current->g + cost(successor,current);
            successor->h[0] = heuristic(successor,goal);
            successor->f[0] = successor->g+successor->h[0];
            successor->h[1] = 10*my_heuristic(successor,goal);
            successor->f[1] = successor->g+successor->h[1];
            string Signature_action = find_signature_action(current,successor,representative_points,dX[i]);
            successor->signature = new_append_signature(current->signature,Signature_action);
            // if(current->x>450 && current->y>=300)
            // {
            //   cout<<successor->x<<' '<<successor->y<<' '<<successor->signature<<'\n';
            // }

            if(inad_closed_map[Hash(successor)]!= 1)
            {
              if(inad_open_map[Hash(successor)]!=1)
              {
                inad_open.push(successor);
                inad_open_map[Hash(successor)] = 1;
                inad_open_map_of_states[Hash(successor)] = successor;
              }
              else if(successor->f < inad_open_map_of_states[Hash(successor)]->f)
              {
                inad_open_map_of_states[Hash(successor)] = successor;
                inad_open.push(successor);
              }
            }

            if(ad_closed_map[Hash(successor)]!= 1)
            {
              if(ad_open_map[Hash(successor)]!=1)
              {
                ad_open.push(successor);
                ad_open_map[Hash(successor)] = 1;
                ad_open_map_of_states[Hash(successor)] = successor;
              }
              else if(successor->f < ad_open_map_of_states[Hash(successor)]->f)
              {
                ad_open_map_of_states[Hash(successor)] = successor;
                ad_open.push(successor);
              }
            }

          }
        }
      }
    }
    else
    {
        node* current = ad_open.top();
        ad_open.pop();
        ad_closed_map[Hash(current)] = 1;
    //   closed_map_of_states[Hash(current)] = current;
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
          // if(current->x>=420 && current->signature=="")
          // cout<<successor->x<<' '<<successor->y<<' '<<successor->signature<<'\n';
          successor->parent = current;
          if(successor->x>0&&successor->x<map_x_size&&successor->y>0&&successor->y<map_y_size)
          {
            // if(current->x>=420 && current->y<=100 && current->signature=="")
            // cout<<successor->x<<' '<<successor->y<<' '<<successor->signature<<'\n';
            if(map[successor->x][successor->y]<collision_threshold)
            {
              successor->g = current->g + cost(successor,current);
              successor->h[0] = heuristic(successor,goal);
              successor->f[0] = successor->g+successor->h[0];
              successor->h[1] = 10*my_heuristic(successor,goal);
              successor->f[1] = successor->g+successor->h[1];
              string Signature_action = find_signature_action(current,successor,representative_points,dX[i]);
              successor->signature = new_append_signature(current->signature,Signature_action);
              // if(current->x>450 && current->y>=300)
              // {
              //   cout<<successor->x<<' '<<successor->y<<' '<<successor->signature<<'\n';
              // }

              if(inad_closed_map[Hash(successor)]!= 1)
              {
                if(inad_open_map[Hash(successor)]!=1)
                {
                  inad_open.push(successor);
                  inad_open_map[Hash(successor)] = 1;
                  inad_open_map_of_states[Hash(successor)] = successor;
                }
                else if(successor->f < inad_open_map_of_states[Hash(successor)]->f)
                {
                  inad_open_map_of_states[Hash(successor)] = successor;
                  inad_open.push(successor);
                }
              }

              if(ad_closed_map[Hash(successor)]!= 1)
              {
                if(ad_open_map[Hash(successor)]!=1)
                {
                  ad_open.push(successor);
                  ad_open_map[Hash(successor)] = 1;
                  ad_open_map_of_states[Hash(successor)] = successor;
                }
                else if(successor->f < ad_open_map_of_states[Hash(successor)]->f)
                {
                  ad_open_map_of_states[Hash(successor)] = successor;
                  ad_open.push(successor);
                }
              }

            }
          }
        }
    }
  }
}
