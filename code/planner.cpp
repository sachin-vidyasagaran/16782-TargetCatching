#include <math.h>
#include <mex.h>
#include <stdio.h>
#include <string.h>
#include <utility>
#include <stdio.h>
#include <string.h>
#include <queue>
#include <unordered_map>
#include <climits>
#include <iostream>
#include <ctime>
#include <cmath>
#include <stack>

#include "aStar.hpp"

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

#define printfunc(...) { mexPrintf(__VA_ARGS__); mexEvalString("drawnow;");}

int runCount = 0;
clock_t start;

/* This function computes the number of steps of the path from the robot to the goal */
int aStar::stepsToSeg()
{
    int i = xyToIndex(goalposeX, goalposeY);     //Initializing to the goalIndex
    int steps = 0;
    while(i != xyToIndex(robotposeX,robotposeY))
    {
        i = cellInfo[i].parent;
        steps++;
    }
    return steps;
}


/* This function backtracks from the goal position to the current robot position
while pushing all intermediate positions to a stack, so that the path from the 
robot to the goal is in the correct sequence*/
void aStar::backTrack()
{

    int i = xyToIndex(goalposeX, goalposeY);     //Initializing to the goalIndex

    while(cellInfo[i].parent != xyToIndex(robotposeX,robotposeY))
    {
        returnPath.push(i);
        i = cellInfo[i].parent;
    }
    returnPath.push(i);
}


/* This function checks if the successor is not in collision and is within the
bounds of the map. It returns 1 if the successor is valid, and 0 if it is not. */
int aStar::sIsValid(int sIndex)
{
    if (map[sIndex] < collision_thresh)
    {
        if (indexToXY(sIndex).first > 1 && indexToXY(sIndex).first < x_size)
        {
            if (indexToXY(sIndex).second > 1 && indexToXY(sIndex).second < y_size)
                return 1;
        }
    }
    return 0;
}

/* This function returns the successor index */
int aStar::sIndex(int s, int nodeIndex)
{
    //get node coordinates from index
    pair<int, int> nodeCoordinates = indexToXY(nodeIndex);
    int successorX = nodeCoordinates.first + successors[0][s];
    int successorY = nodeCoordinates.second + successors[1][s];

    return xyToIndex(successorX, successorY);
}

/* This is the key function of the planner which implements the A* algorithm*/
void aStar::computePath()
{
    while(!this->openList.empty())
    {
        //Pop out the cell with lowest f value
        pair<int, int> node = this->openList.top();     //f-value, cell index

        //Remove it from the openList
        this->openList.pop();

        if (closedList[node.second] == true)    //If the state has already been expanded
            continue;

        //Add it to the closedList
        closedList[node.second] = true;     //cell index, closed

        //Iterate Over the successors
        for(int i=0 ; i<8 ; i++)
        {
            if (!sIsValid(sIndex(i, node.second)))
                continue;

            if (cellInfo[sIndex(i, node.second)].g > cellInfo[node.second].g + this->map[sIndex(i, node.second)]) //If g of new > g of curr + cost of new
            {
                cellInfo[sIndex(i, node.second)].g = cellInfo[node.second].g + this->map[sIndex(i, node.second)];  //Set g of new = g of curr + cost of new
                openList.push(make_pair(cellInfo[sIndex(i, node.second)].g + 1, sIndex(i, node.second)));
                cellInfo[sIndex(i, node.second)].parent = node.second;
            }
        }
    }
}

/* This function computes the cost of the path from the robot to the goal */
int aStar::pathCost()
{
    int i = xyToIndex(goalposeX, goalposeY);
    int cost = 0;
    while(i != xyToIndex(robotposeX,robotposeY))
    {
        cost += (int)map[i];
        i = cellInfo[i].parent;
    }
    return cost;
}

/* This function identifies the section of the target's path that the robot
should aim to intercept the target at. It looks at the time elapsed since 
running the 2D Dijkstra search and accounts for the distance covered by the
target during this time.*/
void aStar::goalFinder()
{
    const int computationTime = (int)ceil(double(clock()-start)/double(CLOCKS_PER_SEC));
    int leastCost = INT_MAX;
    int strategyCost = 0;
    int finalGoalX, finalGoalY;

    for(int trajPoint=0 ; trajPoint < target_steps ; trajPoint ++)
    {
        goalposeX = (int)target_traj[trajPoint];
        goalposeY = (int)target_traj[trajPoint + target_steps];

        if (stepsToSeg() + computationTime < trajPoint)   //If you get to the point on the trajectory faster than the target does
        {   
            strategyCost = pathCost() + (int)(trajPoint-stepsToSeg())*map[xyToIndex(goalposeX,goalposeY)];
            if (strategyCost < leastCost)
            {
                leastCost = strategyCost;
                finalGoalX = goalposeX;
                finalGoalY = goalposeY;
            }
        }
    }
    goalposeX = finalGoalX;
    goalposeY = finalGoalY;
}

/*This is the "main" function of the planner. It outlines the flow of actions
that the robot needs to take in order to catch the target*/
pair<int, int> aStarSearch(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    static aStar a(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps,
            target_traj, targetposeX, targetposeY, curr_time, action_ptr); 

    if (runCount == 1)
    {    
        start = clock();
        a.initStartCell();
        a.computePath();
        a.goalFinder();
        a.backTrack();
    }

    //If you are at the goal, stay at the same spot
    if(robotposeX == a.goalposeX && robotposeY == a.goalposeY)
        return make_pair(robotposeX, robotposeY);

    int nextIndex = a.returnPath.top();
    a.returnPath.pop();

    pair<int, int> nextXY = a.indexToXY(nextIndex);
    return nextXY;
}

/*This function interfaces with the mex function through the action_ptrs to
provide the next position that the robot should travel to*/
static void planner(
        double*	map,
        int collision_thresh,
        int x_size,     //Number of columns
        int y_size,     //Number of rows
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    runCount++;
    pair<int, int> nextXY = aStarSearch(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, action_ptr);
    action_ptr[0] = nextXY.first;
    action_ptr[1] = nextXY.second;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//              MEX FUNCTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )    
{
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}