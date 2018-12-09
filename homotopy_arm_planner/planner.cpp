/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]


/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]


#include <config.h>
#include <astar.h>

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
// 	no plan by default
	*plan = NULL;
	*planlength = 0;

   //for now just do straight interpolation between start and goal checking for the validity of samples

   double distance = 0;
   int i,j;
   for (j = 0; j < numofDOFs; j++){
       if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
           distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
   }
   int numofsamples = (int)(distance/(PI/20));
   if(numofsamples < 2){
       printf("the arm is already at the goal\n");
       return;
   }
   *plan = (double**) malloc(numofsamples*sizeof(double*));
   int firstinvalidconf = 1;
   for (i = 0; i < numofsamples; i++){
       (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
       for(j = 0; j < numofDOFs; j++){
           (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
       }
//        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
       if (false)
       {
           firstinvalidconf = 1;
           printf("ERROR: Invalid arm configuration!!!\n");
       }
   }
   *planlength = numofsamples;

   return;
}

//static void generatePlan(std::vector<hw_2::VertexPtr>& path,
//                         int numofDOFs,
//                         double*** plan,
//                         int* planlength)
//{
//// 	no plan by default
//  *plan = NULL;
//  *planlength = 0;
//
//  *planlength = static_cast<int>(path.size());
//
//  *plan = (double**) malloc((*planlength)*sizeof(double*));
//  for (int i = 0; i < (*planlength); i++)
//    {
//    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
//    for(int j = 0; j < numofDOFs; j++)
//      {
//      (*plan)[i][j] = path[i]->state_.q_[j];
//      }
//    }
//  return;
//}

static void generatePlan(std::vector<homotopy_planner::VertexPtr>& path,
                         int numofDOFs,
                         double*** plan,
                         int* planlength)
{
// 	no plan by default
  *plan = NULL;
  *planlength = 0;

  *planlength = static_cast<int>(path.size());

  *plan = (double**) malloc((*planlength)*sizeof(double*));
  for (int i = 0; i < (*planlength); i++)
    {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
    for(int j = 0; j < numofDOFs; j++)
      {
      (*plan)[i][j] = static_cast<double>(path[i]->state_.q_[j])/DOUBLE_TO_INT_FACTOR;
      }
    }
  return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;


    homotopy_planner::AStar planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad);

    int* start_q = new int[NUM_DOF];
    int* goal_q = new int[NUM_DOF];
    for (int i=0; i<NUM_DOF; ++i)
    {
      start_q[i] = planner.contToDisc(armstart_anglesV_rad[i]);
      goal_q[i] = planner.contToDisc(armgoal_anglesV_rad[i]);
    }
    homotopy_planner::ArmState start_state(start_q);
    homotopy_planner::ArmState goal_state(goal_q);

    if (!planner.IsValidArmConfiguration(start_q, NUM_DOF, map, x_size, y_size) ||
        !planner.IsValidArmConfiguration(goal_q, NUM_DOF, map, x_size, y_size))
    {
      std::cout << "Start or goal in collision! Exiting" << std::endl;
    }

    using namespace std::chrono;
    typedef std::chrono::high_resolution_clock Clock;
    auto start_time = Clock::now();
    if (planner.run(start_state, goal_state))
    {
      auto end_time = Clock::now();
      std::cout << "Planning time: "
                << duration_cast<duration<double> >(end_time - start_time).count() << "s" << std::endl;
    }

  std::vector<homotopy_planner::VertexPtr> path = planner.getPath();

#if DEBUG
//    for (int i=0; i<path.size(); ++i)
//    {
//      for (int j=0; j<numofDOFs; ++j)
//      {
//        std::cout << path[i]->state_.q_[j] << "\t";
//      }
//      std::cout << path[i]->depth_ << std::endl;
//    }
#endif

  std::cout << "planner returned plan of length = " << path.size() << std::endl;

  generatePlan(path, numofDOFs, &plan, &planlength);

    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





