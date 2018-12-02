string append_signature(string signature_state, string signature_action)
{
  if(signature_action[0]!='-')
  {
    // cout<<"positive"<<'\n';
    std::size_t found = signature_state.find(signature_action);
    if(found!=std::string::npos)
    {
      // cout<<"Found"<<'\n';
      if(signature_state[found-1]=='-')
      {
        // cout<<"found negative version"<<'\n';
        signature_state.erase(found-1,2);
      }
    }
    else
    {
      // cout<<"not found"<<'\n';
      signature_state+=signature_action;
      // cout<<signature_state;
    }
    // cout<<signature_state<<'\n';
  }
  else
  {
    // cout<<"negative";
    std::size_t found_negative_version = signature_state.find(signature_action);
    if(found_negative_version!=std::string::npos)
    {
      return signature_state;
    }
    string positive_version_action;
    positive_version_action.push_back(signature_action[1]);
    std::size_t found_positive_version = signature_state.find(positive_version_action);
    if(found_positive_version!=std::string::npos)
    {
      signature_state.erase(found_positive_version,1);
    }
    else
    {
      signature_state+=signature_action;
    }
  }
  // cout<<"BHBJ "<<signature_state;
  return signature_state;
}
string find_signature_action(node *current, node*successor,vector<Point2f> representative_points, int dX)
{
  for(int i=0;i<representative_points.size();i++)
    {
      if(int(representative_points[i].x)==current->x)
      {
        if(current->y>int(representative_points[i].y))
        {
          // return "hi";
          // if(dX==0) return "";
          // else if(dX==1) return std::to_string(i+1);
          if(dX==-1) return "-"+std::to_string(i+1);
        }
      }
    }
  for(int i=0;i<representative_points.size();i++)
    {
      if(int(representative_points[i].x)==successor->x)
      {
        if(successor->y>int(representative_points[i].y))
        {
          // if(dX==1) return "";
          if(dX==1) return std::to_string(i+1);
          // else return "-"+std::to_string(i+1);
        }
      }
    }
  return "";
}
