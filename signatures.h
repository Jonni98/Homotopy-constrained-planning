#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <functional>
std::vector<int> reduced_word(std::vector<int> word, int k)
{
  std::vector<int> reduced_word;
  for(int i=0;i<k;i++)
  {
    if(std::find(word.begin(), word.end(), word[i]*-1)==word.end())
    {
      reduced_word.push_back(word[i]);
    }
  }
  return reduced_word;
}


std::vector<string> PermGenerator(int n, int k,std::vector<int> symbols)
{
    std::vector<int> d(n);
    std::vector<string> a;
    std::iota(d.begin(),d.end(),1);
    // cout << "These are the Possible Permutations: " << endl;
    int vec_num = 0;
    do
    {
        std::vector<int> b;
        for (int i = 0; i < k; i++)
        {
            b.push_back(symbols[d[i]-1]);
        }
        std::vector<int> c;
        c = reduced_word(b,k);
        std::string reducedword;
        for(int j=0;j<c.size();j++)
        {
          // cout<<c[j]<<' ';
          reducedword+=std::to_string(c[j]);
        }
        if(std::find(a.begin(), a.end(), reducedword)==a.end())
        a.push_back(reducedword);
        std::reverse(d.begin()+k,d.end());
    } while (next_permutation(d.begin(),d.end()));

    std::string word;
    return a;
}
void generate_signatures(vector<Point2f> representative_points)
{
  std::vector<int> symbols;
  std::vector<string> signatures;
  std::vector<string> signatures_of_iteration;
  for(int i=1;i<=representative_points.size();i++)
  {
    symbols.push_back(i);
    symbols.push_back(-i);

  }
  // for(int i=0;i<symbols.size();i++)
  // {
  //   cout<<symbols[i]<<'\n';
  // }
  for(int i = 0;i<symbols.size();i++)
    {
      signatures_of_iteration = PermGenerator(symbols.size(),i+1,symbols);
      for(int k =0;k<signatures_of_iteration.size();k++)
      {
        if(std::find(signatures.begin(), signatures.end(), signatures_of_iteration[k])==signatures.end())
          signatures.push_back(signatures_of_iteration[k]);
      }
    }
  // for(int i=0;i<signatures.size();i++)
  // cout<<signatures[i][0]<<'\n';
}
