#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <boost/functional/hash.hpp>

using namespace std;
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


std::size_t Hash1(node* state)
{
  using boost::hash_value;
  using boost::hash_combine;
  std::string string;
  size_t seed = 0;
  hash_combine(seed,state->x* 2654435761);
  hash_combine(seed,state->y* 19349663);
  for(int i=0;i<state->signature.size();i++)
  hash_combine(seed,state->signature[i]*83492791);
  return seed;
}

int main()
{
  node* a,*b;
  a = new node; b = new node;
  a->x = 398; a->y = 281;
  b->x = 401; b->y = 99;
  b->signature = "-12";
  std::cout<<Hash1(a)<<' '<<Hash1(b)<<'\n';
}
