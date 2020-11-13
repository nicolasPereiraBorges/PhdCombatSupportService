#include <stdio.h>
#include <stdlib.h>
#include "float.h" // For DBL_MAX
#include<time.h> // For rand()
#include <math.h> /* sqrt, fabs, fmin, fmax */
//#include "mex.h" /* MEX functions and types */
//#include "matrix.h" /* mwIndex, mwSize */


// Define structs of UAVs, Threats and Parameters
struct uav
{
    double position[2];
    double goal[2];
    double angle;
};

struct threat
{
    double cx;
    double cy;
    double range;
};

struct param
{
    int numberOfEdges;
    double distanceEdges;
    int maxIterations;
    double angleVariation;
    double upperLeft[2];
    double lowerRight[2];
    double distanceThreshold;
};

struct vertex
{
    int id;
    double x;
    double y;
    double distance;
    double angle;
};

struct edge
{
    int idPredecessor;
    int idSucessor;
    double distance;
};

typedef struct uav UAV;
typedef struct threat Threat;
typedef struct param Param;
typedef struct vertex Vertex;
typedef struct edge Edge;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


// Prototypes
//void ReadInput(const mxArray **input, UAV* uav, Threat* threats, int nThreats, Param* param);
double* ApplyRRT(UAV uav, Threat* threats, int nThreats, Param param, int* nPath);
void CheckBounds(Param* param, Threat* threats, int nThreats);
double CalculateDistance(double x1, double y1, double x2, double y2);
void CreateRandomPoint(Param param, double* xOutput, double* yOutput);
int FindNearestVertex(Vertex* vertexes, int numberOfVertexes, int x, int y);
double CalculateAngleBetweenTwoPoints(double x1, double y1, double x2, double y2);
int AngleIsInRange(double x1, double y1, double x2, double y2, double angleRef, Param param);
void ProjectPoint(double x1, double y1, double x2, double y2, double distance, double* xOut, double* yOut);
void GetLineEquation(double x1, double y1, double x2, double y2, double* a, double* b);
double CalculateFx(double a, double b, double x);
int LineIntersectsObstacle(double x1, double y1, double x2, double y2, Threat* threats, int nThreats);
void GetIntersectionBetweenLineAndThreat(double x1, double y1, double x2, double y2, Threat circle, double* x1Out, double* y1Out, double* x2Out, double* y2Out, int* numberOfInterceptions);
int PointInsideDomain(double x, double y, double xlim1, double xlim2, double ylim1, double ylim2);
void ApplyDijktra(Vertex* vertexes, Edge* edges, int idSource, int idGoal);
int GetIndexOfMatrix(int i, int j, int n);
int MinDistance(double dist[], int sptSet[], int nVertexes);
int* Dijkstra(double* graph, int src, int goal, int nVertexes, int* nPath);
void GetPath(int parent[], int j, int outputIndexes[], int* currentIndex);
double* GetPointsOfDijkstra(Vertex* vertexes, int* pathIndexes, int nPath);

// Main function
/* Entry point for the MEX interface */
int main()
{
    UAV uav;
    Param param;
    int nThreats = 7;
    Threat threats[nThreats];

      // Get UAV attributes
    uav.position[0] = 1;
    uav.position[1] = 0;
    uav.goal[0] = 15000;
    uav.goal[1] = 0;
    uav.angle = 0;


    param.numberOfEdges = 5000;
    param.distanceEdges = 0.03;
    param.maxIterations = 15000;
    param.angleVariation = 1.0472;
    param.upperLeft[0] = -500;
    param.upperLeft[1] = 5300;
    param.lowerRight[0] = 15500;
    param.lowerRight[1] = -6540.31669469476;
    param.distanceThreshold = 450;



    threats[0].cx = 2737.56839695844;
    threats[0].cy = -5109.91669469476;
    threats[0].range = 1192;

    threats[1].cx = 7141.13061710690;
    threats[1].cy = 4000.19156848334;
    threats[1].range = 859;

    threats[2].cx = 10978.3140876165;
    threats[2].cy = -3434.62915840442;
    threats[2].range = 855;

    threats[3].cx = 2749.40694811493;
    threats[3].cy = -177.387325581062;
    threats[3].range = 813;

    threats[4].cx = 2886.97315454407;
    threats[4].cy = 2112.73869777410;
    threats[4].range = 831;

    threats[5].cx = 10404.8837365395;
    threats[5].cy = 800.083132449590;
    threats[5].range = 1044;

    threats[6].cx = 13036.9788652146;
    threats[6].cy = 2900.01547395576;
    threats[6].range = 942;



    // Update lower right and upper left if necessary
    //CheckBounds(&param, threats, nThreats);
    int i;
   // printf("Param - nEdges %d DistEdges %.2f MaxIterations %d angle %.2f UpperLeft [%.2f, %.2f] LowerRight [%.2f, %.2f] DistanceThresh %.2f\n", param.numberOfEdges, param.distanceEdges, param.maxIterations, param.angleVariation, param.upperLeft[0], param.upperLeft[1], param.lowerRight[0], param.lowerRight[1], param.distanceThreshold);
    int nPath;
    double* path;

    double x1 = 8331.46683000000;
    double y1 = 127.016710000000;
    double x2 = 9426.08417000000;
    double y2 = -2087.95129000000;
    double angle = -1.73341000000000;

    printf(" Angle is in range - %d\n", AngleIsInRange(x1,y1,x2,y2,angle, param));

    return 0;


    for (i = 0; i < 1; i++)
    {
      //  printf("%d\n", i);
        path = ApplyRRT(uav, threats, nThreats, param, &nPath);
        int j;
       // for (j = 0; j < nPath; j++)
         //   printf("%d %.2f %2f\n", j, path[j], path[j+nPath]);
    }

      mexErrMsgIdAndTxt("Automatic Message","Executed without errors");

   // free(path);
    return 0;
}

/*
// void ReadInput(const mxArray **input, UAV* uav, Threat* threats, int nThreats, Param* param)
// Convert input values to struct UAV Threat* and Param
void ReadInput(const mxArray **input, UAV* uav, Threat* threats, int nThreats, Param* param)
{
    // Store the pointer to the input arrays
    double* uavArray = mxGetPr(input[0]);
    double* threatsArray = mxGetPr(input[1]);
    double* paramArray = mxGetPr(input[2]);
    int i;

    // Get UAV attributes
    uav->position[0] = uavArray[0];
    uav->position[1] = uavArray[1];
    uav->goal[0] = uavArray[2];
    uav->goal[1] = uavArray[3];
    uav->angle = uavArray[4];

    // Get threats attributes
    for (i = 0; i < nThreats; i++)
    {
        threats[i].cx = threatsArray[i];
        threats[i].cy = threatsArray[nThreats+i];
        threats[i].range = threatsArray[(nThreats*2)+i];
    }

    // Get parameters attributes
    param->numberOfEdges = paramArray[0];
    param->distanceEdges = paramArray[1];
    param->maxIterations = paramArray[2];
    param->angleVariation = paramArray[3];
    param->upperLeft[0] = paramArray[4];
    param->upperLeft[1] = paramArray[5];
    param->lowerRight[0] = paramArray[6];
    param->lowerRight[1] = paramArray[7];
    param->distanceThreshold = paramArray[8];
}
*/
// void ApplyRRT(double* path, UAV uav, Threat* threats, int nThreats, Param param)
// Apply Rapidly Exploring Random Tree for calculating path that avoid the threats
double* ApplyRRT(UAV uav, Threat* threats, int nThreats, Param param, int* nPoints)
{
    double x,y, xOut, yOut;
    int maxNumberOfVertexes = param.numberOfEdges+2;
    Vertex vertexes[maxNumberOfVertexes];// = (Vertex*) malloc(sizeof(Vertex) * maxNumberOfVertexes); // idVertex, x, y, distance, angle
    Edge edges[maxNumberOfVertexes];// = (Edge*) malloc(sizeof(Edge) * maxNumberOfVertexes);  // idVertex, idEdge, distanc
    int i;
    int idGoalVertex = 0;

    int numberOfVertexes;
    // First vertex is startPoint
    vertexes[0].id = 0;
    vertexes[0].x = uav.position[0];
    vertexes[0].y = uav.position[1];
    vertexes[0].distance = CalculateDistance(uav.position[0], uav.position[1], uav.goal[0], uav.goal[1]);
    vertexes[0].angle = uav.angle;
    numberOfVertexes = 1;

    int iterations = 0;
    i = 0;
    // For each iteration
    while (i < param.numberOfEdges && iterations < param.maxIterations)
    {
        // Increment the number of iterations
        iterations++;
         // Create random point in scenario
        CreateRandomPoint(param, &x, &y);
        int idClosestVertex = FindNearestVertex(vertexes, numberOfVertexes, x, y);
        //printf("%d - Angle in Range\n", i);
        int inRange = AngleIsInRange(vertexes[idClosestVertex].x, vertexes[idClosestVertex].y, x, y, vertexes[idClosestVertex].angle, param);
        // Check if point is too close to vertex or if angle between closest vertex and point out of range
        if ((inRange == 0) || (vertexes[idClosestVertex].distance < param.distanceThreshold))
            // Out of angle range
            continue;
        // Project line from closest vertex to random point with a distance threshold
        ProjectPoint(vertexes[idClosestVertex].x, vertexes[idClosestVertex].y, x, y, param.distanceThreshold, &xOut, &yOut);
        // Check if edge candidate pass through some threat
        if (LineIntersectsObstacle(vertexes[idClosestVertex].x, vertexes[idClosestVertex].y, xOut, yOut, threats, nThreats)== 1)
            //printf("Intercepts\n");
            continue;
        i++;

        // Add vertex
        double distToGoal = CalculateDistance(xOut, yOut, uav.goal[0], uav.goal[1]);
        vertexes[numberOfVertexes].id = numberOfVertexes;
        vertexes[numberOfVertexes].x = xOut;
        vertexes[numberOfVertexes].y = yOut;
        vertexes[numberOfVertexes].distance = distToGoal;
        vertexes[numberOfVertexes].angle = CalculateAngleBetweenTwoPoints(vertexes[idClosestVertex].x, vertexes[idClosestVertex].y, xOut, yOut);

        if (vertexes[idGoalVertex].distance > distToGoal)
            idGoalVertex = numberOfVertexes;


         //Add Edge
        //printf("%d - Update edge\n", i);
        edges[numberOfVertexes-1].distance = param.distanceThreshold;
        edges[numberOfVertexes-1].idPredecessor = idClosestVertex;
        edges[numberOfVertexes-1].idSucessor = numberOfVertexes;

        //printf("%d\t%d\t%.3f\t%.3f\t%.3f\t%.3f\n",i, numberOfVertexes-1, vertexes[idClosestVertex].x, vertexes[idClosestVertex].y, xOut, yOut);
        numberOfVertexes++;
       // if (distToGoal <= param.distanceThreshold)
       //     break;

        // printf("No intercepts\n");
    }


    double* costMatrix = (double*) malloc(sizeof(double) * (numberOfVertexes*numberOfVertexes));  // idVertex, idEdge, distanc
    for (i = 0; i < numberOfVertexes-1; i++)
    {
        int idPredecessor = edges[i].idPredecessor;
        int idSuccessor = edges[i].idSucessor;
        costMatrix[GetIndexOfMatrix(idPredecessor, idSuccessor, numberOfVertexes)] = edges[i].distance;
    }

    int nPathIndexes;
    //printf("idGoalVertex - %d %.2f\n", idGoalVertex, vertexes[idGoalVertex].distance);
    int* pathIndexes = Dijkstra(costMatrix, 0, idGoalVertex, numberOfVertexes, &nPathIndexes);
    int j;
  //  for (j = 0; j < nPathIndexes; j++)
  //          printf("Indexes %d - %d\n", j,pathIndexes[j]);

    double* points = GetPointsOfDijkstra(vertexes, pathIndexes, nPathIndexes);

    //printf("\n\n\n\n\n");
    //for (j = 0; j < nPathIndexes; j++)
    //       printf("%d\t%.3f\t%.2f\n", j, points[j], points[j+nPathIndexes]);

    (*nPoints) = nPathIndexes;
    //free(edges);
   // free(vertexes);
    free(costMatrix);
    return points;

}


void CheckBounds(Param* param, Threat* threats, int nThreats)
{

    int i;
    for (i = 0; i < nThreats; i++)
    {
        Threat threat = threats[i];
        //printf("%d - cx %.2f cy %.2f range %.2f\n", i, threat.cx, threat.cy, threat.range);
        if (threat.cx - threat.range < param->upperLeft[0])
            param->upperLeft[0] = threat.cx - (threat.range*1.2);
        if (threat.cx + threat.range > param->lowerRight[0])
            param->lowerRight[0] = threat.cx + (threat.range*1.2);
        if (threat.cy + threat.range > param->upperLeft[1])
            param->upperLeft[1] = threat.cy + (threat.range*1.2);
        if (threat.cy - threat.range < param->lowerRight[1])
            param->lowerRight[1] = threat.cy - (threat.range*1.2);

    }
}

// double CalculateDistance(double x1, double y1, double x2, double y2)
// Calculate euclidian distance between two points
double CalculateDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

// void CreateRandomPoint(Param param, double* xOutput, double* yOutput)
// Create random point in the range of upperLeft and lowerRight
void CreateRandomPoint(Param param, double* xOutput, double* yOutput)
{
    double r = (double)rand() / (double)RAND_MAX ;
    *xOutput = (param.lowerRight[0] - param.upperLeft[0]) * r + param.upperLeft[0];
    r = (double)rand() / (double)RAND_MAX ;
    *yOutput =(param.upperLeft[1] - param.lowerRight[1]) * r + param.lowerRight[1];
}

// int FindNearestVertex(double** vertexes, int numberOfVertexes, int x, int y)
// Get index of vertex (int vertexes) that is close to x and y
int FindNearestVertex(Vertex* vertexes, int numberOfVertexes, int x, int y)
{
    double minDist = DBL_MAX;
    int indexMin = -1;
    int i;

    // For each vertex
    for (i = 0; i < numberOfVertexes; i++)
    {
        // Calculate distance from point to vertex
        double dist = CalculateDistance(x,y, vertexes[i].x, vertexes[i].y);
        // If distance is lower than min distance
        if (dist < minDist)
        {
            // Update min distance
            minDist = dist;
            indexMin = i;
        }
    }
    return indexMin;
}

// double CalculateAngleBetweenTwoPoints(double x1, double y1, double x2, double y2)
// Calculate angle in radians between two points
double CalculateAngleBetweenTwoPoints(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1, x2-x1);
}

// int AngleIsInRange(double x1, double y1, double x2, double y1, double angleRef)
// Check if current angle is in range limited by angle ref
int AngleIsInRange(double x1, double y1, double x2, double y2, double angleRef, Param param)
{
    // Calculate angle with the new point
    double angle = CalculateAngleBetweenTwoPoints(x1,y1,x2,y2);
    //printf("Angle - %.2f - AngleRef - %.2f\n", angle, angleRef);
    double variation = param.angleVariation;
    if (angle < 0 && angleRef < 0)
    {
        angle = fabs(angle);
        angleRef = fabs(angleRef);
    }
    else if ((angle < 0 && angleRef > 0) || (angle > 0 && angleRef < 0))
    {
        variation/=2;
    }

    double diff = MAX(angle,angleRef) - MIN(angle, angleRef);
    // Check if new angle is not in range of reference angle
    if (diff > variation)
        return 0;

    return 1;
}

// void ProjectPoint(double x1, double y1, double x2, double y2, double distance, double* xOut, double* yOut)
// Project point from [x1,y1] to [x2,y2] in a straigth with a distance threshold
void ProjectPoint(double x1, double y1, double x2, double y2, double distance, double* xOut, double* yOut)
{

    int reverse = 0;
    // if p1 and p2 have same x and different y
    if ((x1 == x2) && (y1 != y2))
    {
        // Invert x and y
        double aux = x1;
        x1 = y1;
        y1 = aux;

        aux = x2;
        x2 = y1;
        y2 = aux;

        reverse = 1;
    }

    double a,b;
    // Get line equation
    GetLineEquation(x1, y1, x2, y2, &a, &b);
    // Calculate fx at the projected distance
    double xres_aux = x1+((x2-x1)* distance) / CalculateDistance(x1, y1, x2, y2);
    double yres_aux = CalculateFx(a, b, xres_aux);

    if (reverse == 1)
    {
        (*xOut) = yres_aux;
        (*yOut) = xres_aux;
    }

    else
    {
        (*xOut) = xres_aux;
        (*yOut) = yres_aux;
    }

}

// void GetLineEquation(double x1, double y1, double x2, double y2, double* a, double* b)
// Calculate line equation that pass trought [x1,y1] and [x2,y2] on the form
void GetLineEquation(double x1, double y1, double x2, double y2, double* a, double* b)
{
    // Calculate angular coefficient
    (*a) = (y1-y2)/(x1-x2);
    // y - y0 = + m(x - x0)
    (*b) = y1 - (*a)*x1;
}

// double CalculateFx(double a, double b, double x)
// Calculate F(x) given line equation in format f(x) = a*x + b
double CalculateFx(double a, double b, double x)
{
    return a*x + b;
}


// int LineIntersectsObstacle(double x1, double y1, double x2, double y2, Threat* threats, int nThreats)
// Check if line intersects with some obstacle
int LineIntersectsObstacle(double x1, double y1, double x2, double y2, Threat* threats, int nThreats)
{
    if (nThreats == 0)
        return 0;

    int i, numberOfInterceptions;
    double a, b, xInt1, yInt1, xInt2, yInt2;
    // Calculate line between start point and goal
    GetLineEquation(x1, y1, x2, y2, &a, &b);
    // for each threat
    for (i = 0; i < nThreats; i++)
    {
        // Get interception between line and circle
        GetIntersectionBetweenLineAndThreat(x1, y1, x2, y2, threats[i], &xInt1, &yInt1, &xInt2, &yInt2, &numberOfInterceptions);
        if (numberOfInterceptions == 0)
            continue;
        else if (numberOfInterceptions == 1) // Single interceptions
        {
            if (PointInsideDomain(xInt1, yInt1, x1, x2, y1, y2) == 1)
                return 1;
        }
        else // 2 interceptions
        {
            if (PointInsideDomain(xInt1, yInt1, x1, x2, y1, y2) == 1)
                return 1;
            if (PointInsideDomain(xInt2, yInt2, x1, x2, y1, y2) == 1)
                return 1;
        }
    }
    return 0;

}

// void GetIntersectionBetweenLineAndThreat(double x1, double y1, double x2, double y2, Threat circle, double* x1Out, double* y1Out, double* x2Out, double* y2Out, int* numberOfInterceptions)
// Get interception points between line and circunference
void GetIntersectionBetweenLineAndThreat(double x1, double y1, double x2, double y2, Threat threat, double* x1Out, double* y1Out, double* x2Out, double* y2Out, int* numberOfInterceptions)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double A = dx*dx + dy*dy;
    double B = 2*(dx*(x1-threat.cx) + dy*(y1-threat.cy));
    double C = (x1-threat.cx)*(x1-threat.cx) + (y1-threat.cy)*(y1-threat.cy) - threat.range*threat.range;
    double det = B*B - 4*A*C;


    if ((A <= 0.0000001) || (det < 0)) // No interse
        (*numberOfInterceptions) = 0;
    else if (det == 0) // One solution
    {
        (*numberOfInterceptions) = 1;
        double t = -B/(2*A);
        (*x1Out) = x1 + t * dx;
        (*y1Out) = y1 + t * dy;
        (*x2Out) = x1 + t * dx;
        (*y2Out) = y1 + t * dy;
    }
    else // Two solutions
    {
        (*numberOfInterceptions) = 2;
        double t = ((-B + sqrt(det))/(2*A));
        (*x1Out) = x1 + t * dx;
        (*y1Out) = y1 + t * dy;
        t = ((-B - sqrt(det))/(2*A));
        (*x2Out) = x1 + t * dx;
        (*y2Out) = y1 + t * dy;
    }
}

// int PointInsideDomain(double x, double y, double xlim1, double xlim2, double ylim1, double ylim2)
// Check if point is inside a domain (limit)
int PointInsideDomain(double x, double y, double xlim1, double xlim2, double ylim1, double ylim2)
{
    double x1,y1,x2,y2;

    // Get xBounds
    x1 = MIN(xlim1, xlim2);
    x2 = MAX(xlim1, xlim2);
    // Get y bounds
    y1 = MIN(ylim1, ylim2);
    y2 = MAX(ylim1, ylim2);
    // Check domain
    if( (x >= x1)  && (x <= x2) && (y >= y1) && (y <= y2))
        return 1;

    return 0;

}

int GetIndexOfMatrix(int i, int j, int n)
{

    return n * i + j;
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int MinDistance(double dist[], int sptSet[], int nVertexes)
{
   // Initialize min value
   double min = DBL_MAX, min_index;
   int v;
   for (v = 0; v < nVertexes; v++)
     if (sptSet[v] == 0 && dist[v] <= min)
     {
         min = dist[v];
         min_index = v;
     }

   return min_index;
}



// void GetPath(int parent[], int j, int outputIndexes[], int* currentIndex)
// Get full path from source to goal given given parent array
void GetPath(int parent[], int j, int outputIndexes[], int* currentIndex)
{
    // Base Case : If j is source
    if (parent[j]==-1)
        return;

    outputIndexes[(*currentIndex)++] = j;
    GetPath(parent, parent[j], outputIndexes, currentIndex);

   // printf("%d\n", j);
}
// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
int* Dijkstra(double* graph, int src, int goal, int nVertexes, int* nPath)
{
     double dist[nVertexes];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
    // Parent array to store shortest path tree
     int parent[nVertexes];
     int sptSet[nVertexes]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
     int i;
     // Initialize all distances as INFINITE and stpSet[] as false
     for (i = 0; i < nVertexes; i++)
     {
        parent[i] = -1;
        dist[i] = DBL_MAX;
        sptSet[i] = 0;
     }


     // Distance of source vertex from itself is always 0
     dist[src] = 0;
     int count;
     // Find shortest path for all vertices
     for ( count = 0; count < nVertexes-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = MinDistance(dist, sptSet, nVertexes);
       //printf("U - %d\n", u);

       // Mark the picked vertex as processed
       sptSet[u] = 1;


       int indexMatrix, v;
       // Update dist value of the adjacent vertices of the picked vertex.
       for (v = 0; v < nVertexes; v++)
       {
        indexMatrix = GetIndexOfMatrix(u,v,nVertexes);
        // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[indexMatrix] && dist[u] != DBL_MAX
                                       && dist[u]+graph[indexMatrix] < dist[v])
                                       {
                                           //printf("Parent[%d] = %d\n",v,u);
                                           parent[v]  = u;
                                           dist[v] = dist[u] + graph[indexMatrix];
                                       }

       }

     }

     int pathIndexes[nVertexes];
     (*nPath) = 0;

     GetPath(parent, goal, pathIndexes, nPath);
     //printf("11.2\n");
     pathIndexes[(*nPath)++] = src;

    return pathIndexes;
}

double* GetPointsOfDijkstra(Vertex* vertexes, int* pathIndexes, int nPath)
{
    double* outputPath = (double*) malloc (sizeof(double) * (nPath*2));
    int i;

    for (i = 0; i < nPath; i++)
    {
        outputPath[i] = vertexes[pathIndexes[nPath-1-i]].x;
        outputPath[i+nPath] = vertexes[pathIndexes[nPath-1-i]].y;
    }

    return outputPath;

}
/*
double* ReducePath(double* points, int nPath, Threat* threats, int nThreats, int* nReducedPath)
{
    double* reducedPath = (double*) malloc (sizeof(double) * (nPath*2));
    int i,j;

    for (i = 0; i < nPath; i++)
    {
        j = nPath - 1;
        while (j > (i+1))
        {
           int intercepts = LineIntersectsObstacle(points())
        }
    }

}
*/
