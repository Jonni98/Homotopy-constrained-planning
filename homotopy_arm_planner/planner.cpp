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
#define	PLAN_ENDEFFECTOR_OUT	plhs[1]
#define	PLANLENGTH_OUT	plhs[2]


#include <config.h>
#include <astar.h>


static void generatePlan(std::vector<homotopy_planner::VertexPtr>& path,
                         std::vector<homotopy_planner::DiscreteArmPlanner::Point2D>& end_effector_path,
                         int numofDOFs,
                         double*** plan,
                         double*** plan_endeffector,
                         int* planlength)
{
// 	no plan by default
  *plan = NULL;
  *plan_endeffector = NULL;
  *planlength = 0;

  *planlength = static_cast<int>(path.size());

  *plan = (double**) malloc((*planlength)*sizeof(double*));
  *plan_endeffector = (double**) malloc((*planlength)*sizeof(double*));
  for (int i = 0; i < (*planlength); i++)
    {
    (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
    (*plan_endeffector)[i] = (double*) malloc(2*sizeof(double));
    for(int j = 0; j < numofDOFs; j++)
      {
      (*plan)[i][j] = static_cast<double>(path[i]->state_.q_[j])/DOUBLE_TO_INT_FACTOR;
      }
      (*plan_endeffector)[i][0] = end_effector_path[i].x_;
      (*plan_endeffector)[i][1] = end_effector_path[i].y_;
    }
  return;
}

void assignDefaultOutput(mxArray *plhs[], double* start_q, int numofDOFs)
{
  PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
  PLAN_ENDEFFECTOR_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
  double* plan_out = mxGetPr(PLAN_OUT);
  double* plan_endeffector_out = mxGetPr(PLAN_ENDEFFECTOR_OUT);
  //copy the values
  int i,j;
  for (j = 0; j < numofDOFs; j++)
  {
    plan_out[j] = start_q[j];
  }
  for (j = 0; j < 2; j++)
  {
    plan_endeffector_out[j] = 0.0;
  }
  PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL);
  int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
  *planlength_out = 2;
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
    } else if (nlhs != 3) {
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
    double** plan_endeffector = NULL;
    int planlength = 0;


    homotopy_planner::AStar planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad);

    int* start_q = new int[NUM_DOF];
    int* goal_q = new int[NUM_DOF];
    for (int i=0; i<NUM_DOF; ++i)
    {
      start_q[i] = planner.contToDisc(armstart_anglesV_rad[i]);
      goal_q[i] = planner.contToDisc(armgoal_anglesV_rad[i]);
    }
    
    // Signatures
    int goal_sign[] = {-2};
    std::vector<int> goal_sign_vec (goal_sign, goal_sign + sizeof(goal_sign) / sizeof(int) );
  
    homotopy_planner::ArmState start_state(start_q);
    homotopy_planner::ArmState goal_state(goal_q, goal_sign_vec);

    if (!planner.IsValidArmConfiguration(start_q, NUM_DOF, map, x_size, y_size))
    {
      std::cout << "Start in collision! Exiting" << std::endl;
      assignDefaultOutput(plhs, armstart_anglesV_rad, numofDOFs);
      return;
    }

    if (!planner.IsValidArmConfiguration(goal_q, NUM_DOF, map, x_size, y_size))
    {
      std::cout << "Goal in collision! Exiting" << std::endl;
      assignDefaultOutput(plhs, armstart_anglesV_rad, numofDOFs);
      return;
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
  std::vector<homotopy_planner::DiscreteArmPlanner::Point2D> end_effector_path = planner.getEndEffectorPath();

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

  generatePlan(path,
               end_effector_path,
               numofDOFs,
               &plan,
               &plan_endeffector,
               &planlength);

    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
        PLAN_ENDEFFECTOR_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
        double* plan_out = mxGetPr(PLAN_OUT);
        double* plan_endeffector_out = mxGetPr(PLAN_ENDEFFECTOR_OUT);
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
            for (j = 0; j < 2; j++)
            {
              plan_endeffector_out[j*planlength + i] = plan_endeffector[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)2, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
        PLAN_ENDEFFECTOR_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL);
        double* plan_out = mxGetPr(PLAN_OUT);
        double* plan_endeffector_out = mxGetPr(PLAN_ENDEFFECTOR_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }
        plan_endeffector_out[0] = 0.0;
        plan_endeffector_out[1] = 0.0;
        plan_endeffector_out[2] = 0.0;
        plan_endeffector_out[3] = 0.0;
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}





