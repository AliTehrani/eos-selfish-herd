/*
 * tGame.cpp
 *
 * This file is part of the aBeeDa Swarm Evolution project.
 *
 * Copyright 2012 Randal S. Olson, Arend Hintze.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tGame.h"

#define cPI 3.14159265

// simulation-specific constants
#define totalStepsInSimulation  10000
#define gridX                   30.0
#define gridY                   30.0
#define gridXAcross             2.0 * gridX
#define gridYAcross             2.0 * gridY
#define collisionDist           10.0 * 10.0
#define boundaryDist            (gridX - sqrt(collisionDist)/2.0)
#define boundaryVision		(gridX - sqrt(visionRange))
#define V                       10
#define dTheta                  6.0
#define dTime                   0.1

#define nodesAgents		12//states[0-11] other agents
#define nodesBoundary           12//states[12-23] walls
#define visionRange	        60.0 * 60.0
#define visionAngle    		360.0 / 2.0
#define agentSensors            24


// precalculated lookup tables for the game
double cosLookup[360];
double sinLookup[360];
double atan2Lookup[2400][2400];

tGame::tGame() { }

tGame::~tGame() { }

// runs the simulation for the given agent(s)
string tGame::executeGame(vector<tAgent*> swarmAgents, FILE *data_file, bool &report, long &numberCollision, double &avgSwarmFitness, int killDelay, int deme)
{
  string reportString = "";
  enum Action {stay=0, left, right, move};
  Action action;
  //  int i, j;
  int numberCollisions=0;
  double xTemp, yTemp;
  ostringstream sstm;
  sstm << gridX << ", " << gridY << ", " << sqrt(collisionDist) << ", " << swarmSize << " ," << totalStepsInSimulation << ", " << nodesAgents << ", "  << sqrt(visionRange) << ", " << visionAngle <<'\n';
//Initial placement
  initialPlacement(swarmAgents);

//  Iterating...
  for (int step=0; step<totalStepsInSimulation; step++){
    
    senseStates(swarmAgents);
    
//Writing to string
    printToString(swarmAgents, step, sstm);
    
    for (int j=0; j<swarmSize; j++){
      swarmAgents[j]->updateStates();
      action = static_cast<Action>((int)(swarmAgents[j]->states[(maxNodes-2)]&1)*2+(int)(swarmAgents[j]->states[maxNodes-1]&1));
      switch (action){
      case stay:{
	swarmAgents[j]->fitness -= penaltyStay;
	continue;
      }
      case left:{
	swarmAgents[j]->direction = (int)(swarmAgents[j]->direction+dTheta)%360;
	swarmAgents[j]->fitness -= penaltyTurn;
	break;
      }
      case right:{
	swarmAgents[j]->direction = (int)(swarmAgents[j]->direction-dTheta+360)%360;
	swarmAgents[j]->fitness -= penaltyTurn;
	break;
      }
      case move:{
      }
      }
      
      xTemp = swarmAgents[j]->xPos + V*cos(swarmAgents[j]->direction/180.0*cPI)*dTime;
      yTemp = swarmAgents[j]->yPos + V*sin(swarmAgents[j]->direction/180.0*cPI)*dTime;
      
//Walls
      wallsCheck(xTemp, yTemp, swarmAgents[j]->xPos, swarmAgents[j]->yPos, swarmAgents[j]->direction, swarmAgents[j]->fitness);
    }
//Collisions
    for (int m=0; m<swarmSize-1; m++){
      for (int n=m+1; n<swarmSize; n++){
	double distance2 = calcDistanceSquared(swarmAgents[m]->xPos, swarmAgents[m]->yPos, swarmAgents[n]->xPos, swarmAgents[n]->yPos);
	if (distance2 <= collisionDist){
	  numberCollisions++;
	  swarmAgents[m]->fitness -= penaltyCollision;
	  swarmAgents[n]->fitness -= penaltyCollision;
//Bounce back

	  action = static_cast<Action>((int)(swarmAgents[m]->states[maxNodes-1]&1));
	  if (action != stay)
	    bounceBack(swarmAgents[m]->xPos, swarmAgents[m]->yPos, swarmAgents[m]->direction, swarmAgents[n]->xPos, swarmAgents[n]->yPos);
	  action = static_cast<Action>((int)(swarmAgents[n]->states[maxNodes-1]&1));
	  if (action != stay)
	    bounceBack(swarmAgents[n]->xPos, swarmAgents[n]->yPos, swarmAgents[n]->direction, swarmAgents[m]->xPos, swarmAgents[m]->yPos);
	  
//No overlap
/*	    action = static_cast<Action>((int)(swarmAgents[m]->states[maxNodes-2])*2+(int)(swarmAgents[m]->states[maxNodes-1]));
	    if (action!=stay)
	      noOverlap(swarmAgents[m]->xPos, swarmAgents[m]->yPos, swarmAgents[m]->direction);
	    
            action = static_cast<Action>((int)(swarmAgents[n]->states[maxNodes-2])*2+(int)(swarmAgents[n]->states[maxNodes-1]));
            if (action!=stay)
	      noOverlap(swarmAgents[n]->xPos, swarmAgents[n]->yPos, swarmAgents[n]->direction);
*/
	}
      }
    }
  }

  for (int j=0; j<swarmSize; j++){
    if (swarmAgents[j]->fitness <= 0)
      swarmAgents[j]->fitness = 5e-6;
  }

  //if (data_file!=NULL){
  // output fitness and # of collisions to data_file

  avgSwarmFitness = 0.0;
  for (int j = 0; j < swarmSize; j++){
    avgSwarmFitness += swarmAgents[j]->fitness;
  }
  avgSwarmFitness /= (double)swarmSize;
  //fprintf(data_file, "%d,%d,%f,%d\n", swarmAgents[0]->born, deme, avgSwarmFitness, numberCollisions);
  //}
  
  //if (numberCollisions < 10){
  report = true;
  //}
  /*for (int j=0; j<swarmSize; j++){
    if (swarmAgents[j]->fitness < 3000)
    report = false;
    }*/
  reportString = sstm.str();
  return reportString;
}

//Initial placement
void tGame::initialPlacement(vector<tAgent*> swarmAgents){
  for (int j=0; j<swarmSize; j++){
    //check if positions overlap
    bool repeat=false;
    do{
      swarmAgents[j]->xPos = rand()%int(gridX-sqrt(collisionDist)-1)+int(sqrt(collisionDist)/2.0)+1;
      swarmAgents[j]->yPos = rand()%int(gridY-sqrt(collisionDist)-1)+int(sqrt(collisionDist)/2.0)+1;
      repeat=false;
      for (int m=0; (m<j)&&(repeat==false); m++){
	if (calcDistanceSquared(swarmAgents[j]->xPos, swarmAgents[j]->yPos, swarmAgents[m]->xPos, swarmAgents[m]->yPos)<=collisionDist){
	  repeat=true;
      }
      }
    }while(repeat==true);
    swarmAgents[j]->direction = (rand()%360);
    swarmAgents[j]->fitness = fitnessInit;
    swarmAgents[j]->setupPhenotype();
  }
}

//Writing to string
void tGame::printToString(vector<tAgent*> swarmAgents, int& step, ostringstream& sstm){
  for (int j=0; j<swarmSize; j++){
    sstm << step << ", " << swarmAgents[j]->xPos << ", " << swarmAgents[j]->yPos << ", " << swarmAgents[j]->direction << ", " << swarmAgents[j]->fitness;
    for (int l=0; l<agentSensors; l++){
      sstm << ", " << (swarmAgents[j]->states[l]&1);
    }
    sstm << '\n';
  }
}

//bounce back for walls
void tGame::wallsCheck(double xTemp, double yTemp, double &xPos, double &yPos, double &direction, double &fitness){
  if (xTemp > boundaryDist){
    direction = (int)(180-direction+360) % 360;
    xPos = 2*boundaryDist - xTemp;
    fitness -= penaltyWall;
  }
  else if (xTemp < sqrt(collisionDist)/2.0){
    direction = (int)(180-direction+360) % 360;
    xPos = sqrt(collisionDist) - xTemp;
    fitness -= penaltyWall;
  }
  if (yTemp > boundaryDist){
    direction = (int)(-direction+360) % 360;
    yPos = (2.0*boundaryDist) - yTemp;
    fitness -= penaltyWall;
  }
  else if (yTemp < sqrt(collisionDist)/2.0){
    direction = (int)(-direction+360) % 360;
    yPos = sqrt(collisionDist) - yTemp;
    fitness -= penaltyWall;
  }
  else if ((xTemp>=sqrt(collisionDist)/2.0)&&(xTemp<=boundaryDist)){
    xPos = xTemp;
    yPos = yTemp;
    fitness += reward;
  }
}

//no overlap with walls
void tGame::wallsCheck_2(double xTemp, double yTemp, double &xPos, double &yPos, double &direction, double &fitness){
  if (xTemp >= boundaryDist)
    xPos = boundaryDist;
  else if (xTemp <= sqrt(collisionDist)/2.0)
    xPos = sqrt(collisionDist)/2.0;
  if (yTemp >= boundaryDist)
    yPos = boundaryDist;
  else if (yTemp <= sqrt(collisionDist)/2.0)
    yPos = sqrt(collisionDist)/2.0;
  else if ((xTemp>=sqrt(collisionDist)/2.0)&&(xTemp<=boundaryDist)){
    xPos = xTemp;
    yPos = yTemp;
    fitness += reward;
  }
}

//Bounce back
void tGame::bounceBack(double &x1, double &y1, double &theta1, double x2, double y2){
  double gamma = atan2(y1-y2, x1-x2);
  double bb = 2*gamma - theta1/180.0*cPI + cPI;
  if (cos(theta1/180.0*cPI-gamma)<0){
    double xTemp = x1 - V*dTime*cos(theta1/180.0*cPI);
    double yTemp = y1 - V*dTime*sin(theta1/180.0*cPI);
    theta1 = int(bb*180.0/cPI + 360*2) % 360;
    //Walls: bounce back
    if (xTemp > boundaryDist){
      theta1 = (int)(180-theta1+360) % 360;
      x1 = 2*boundaryDist - xTemp;
    }
    else if (xTemp < sqrt(collisionDist)/2.0){
      theta1 = (int)(180-theta1+360) % 360;
      x1 = sqrt(collisionDist) - xTemp;
    }
    if (yTemp > boundaryDist){
      theta1 = (int)(-theta1+360) % 360;
      y1 = (2.0*boundaryDist) - yTemp;
    }
    else if (yTemp < sqrt(collisionDist)/2.0){
      theta1 = (int)(-theta1+360) % 360;
      y1 = sqrt(collisionDist) - yTemp;
    }
    else if ((xTemp>=sqrt(collisionDist)/2.0)&&(xTemp<=boundaryDist)){
      x1 = xTemp;
      y1 = yTemp;
    }
  }
}

//No overlap collision
void tGame::noOverlap(double &xPos, double &yPos, double direction){
  double xTemp = xPos - (V*dTime)*cos(direction/180.0*cPI);
  double yTemp = yPos - (V*dTime)*sin(direction/180.0*cPI);

  if ((xTemp <= boundaryDist)&&(xTemp >= sqrt(collisionDist)/2.0)&&(yTemp <= boundaryDist)&&(yTemp >= sqrt(collisionDist)/2.0)){
    xPos -= xTemp;
    yPos -= yTemp;
  }
}

void tGame::senseStates(vector<tAgent*> swarmAgents){
  //Filling states vector
  for (int j=0; j<swarmSize; j++){
    if ((swarmAgents[j]->direction<0)||(swarmAgents[j]->direction>360))
      cout<< swarmAgents[j]->direction << '\n';
    //zeroing states vector
    for (int l=0; l<nodesBoundary+nodesAgents; l++){
      swarmAgents[j]->states[l]=0;
    }
    //Sensing other agents
    for (int k=0; k<swarmSize; k++){
      if (j==k)
	continue;
      double distance = calcDistanceSquared(swarmAgents[j]->xPos, swarmAgents[j]->yPos, swarmAgents[k]->xPos, swarmAgents[k]->yPos);
      if (distance < visionRange){
	int relativeAngle=calcAngle(swarmAgents[j]->xPos, swarmAgents[j]->yPos, swarmAgents[j]->direction, swarmAgents[k]->xPos, swarmAgents[k]->yPos);

	double blockedSightAngle = asin(sqrt(collisionDist/(4.0*distance)))*180.0/cPI;
	int initAngle = int(relativeAngle - blockedSightAngle + 90)%360;
	int sensePosInit = (initAngle)/(visionAngle/nodesAgents);
	int sensePosFin = (initAngle + 2.0*blockedSightAngle)/(visionAngle/nodesAgents);
	if (sensePosInit < 0)
	  sensePosInit = 0;
	if (sensePosFin >= nodesAgents)
	  sensePosFin = nodesAgents - 1;
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l] = 1;
	}
      }
    }

    //Sensing domain boundaries
    int boundaryAngle;
    int sensePosInit=nodesAgents;
    int sensePosFin= nodesAgents + nodesBoundary - 1;
    int visionAngleInit=((int)swarmAgents[j]->direction - 90+360)%360;
    int visionAngleFin=((int)swarmAgents[j]->direction + 90)%360;
    //Rightside wall
    if (swarmAgents[j]->xPos>=boundaryVision){
      boundaryAngle=acos((gridX-swarmAgents[j]->xPos)/sqrt(visionRange))*180.0/cPI;
      
      if ((swarmAgents[j]->direction<=boundaryAngle+90)&&(swarmAgents[j]->direction>=90-boundaryAngle)){
	sensePosFin -= (int)((visionAngleFin-boundaryAngle)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
      else if ((swarmAgents[j]->direction>=360-boundaryAngle-90)&&(swarmAgents[j]->direction<=360-90+boundaryAngle)){
	sensePosInit += (int)((360-boundaryAngle-visionAngleInit)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	    swarmAgents[j]->states[l]=1;
	}
      }
      else if ((swarmAgents[j]->direction<90-boundaryAngle)||(swarmAgents[j]->direction>360-90+boundaryAngle)){
	sensePosInit += (int)((360-boundaryAngle-visionAngleInit)/(visionAngle/nodesBoundary));
	sensePosFin -= (int)((visionAngleFin-boundaryAngle)/(visionAngle/nodesBoundary));
	  for (int l=sensePosInit; l<=sensePosFin; l++){
	    swarmAgents[j]->states[l]=1;
	  }
      }
    }
    //Upside wall
    sensePosInit=nodesAgents;
    sensePosFin= nodesAgents + nodesBoundary - 1;
    if (swarmAgents[j]->yPos>=boundaryVision){
      boundaryAngle=acos((gridY-swarmAgents[j]->yPos)/sqrt(visionRange))*180.0/cPI;
      
      if ((swarmAgents[j]->direction>=90-boundaryAngle+90)&&(swarmAgents[j]->direction<=90+boundaryAngle+90)){
	sensePosFin -= (int)((visionAngleFin-90-boundaryAngle)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
      else if ((swarmAgents[j]->direction<=boundaryAngle)||(swarmAgents[j]->direction>=360-boundaryAngle)){
	sensePosInit += (int)((180-visionAngleFin+90-boundaryAngle)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
      else if ((swarmAgents[j]->direction>boundaryAngle)&&(swarmAgents[j]->direction<180-boundaryAngle)){
	sensePosInit += (int)((180-visionAngleFin+90-boundaryAngle)/(visionAngle/nodesBoundary));
	sensePosFin -= (int)((visionAngleFin-90-boundaryAngle)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
    }
    //Leftside wall
    sensePosInit=nodesAgents;
    sensePosFin= nodesAgents + nodesBoundary - 1;
    if (swarmAgents[j]->xPos<sqrt(visionRange)){
      boundaryAngle=acos((swarmAgents[j]->xPos)/sqrt(visionRange))*180.0/cPI;

      if ((swarmAgents[j]->direction>=-boundaryAngle+90)&&(swarmAgents[j]->direction<=360+boundaryAngle-90)){
	if ((swarmAgents[j]->direction<=180+boundaryAngle+90)&&(swarmAgents[j]->direction>=180-boundaryAngle+90)){
	  sensePosFin -= (int)((swarmAgents[j]->direction+90-boundaryAngle-180)/(visionAngle/nodesBoundary));
	  for (int l=sensePosInit; l<=sensePosFin; l++){
	    swarmAgents[j]->states[l]=1;
	  }
	}
	else if ((swarmAgents[j]->direction>=180-boundaryAngle-90)&&(swarmAgents[j]->direction<=180+boundaryAngle-90)){
	  sensePosInit += (int)((180-boundaryAngle-(swarmAgents[j]->direction-90))/(visionAngle/nodesBoundary));
	  for (int l=sensePosInit; l<=sensePosFin; l++){
	    swarmAgents[j]->states[l]=1;
	  }
	}
	else if ((swarmAgents[j]->direction>180+boundaryAngle-90)&&(swarmAgents[j]->direction<180-boundaryAngle+90)){
	  sensePosInit += (int)((180-boundaryAngle-(swarmAgents[j]->direction-90))/(visionAngle/nodesBoundary));
	  sensePosFin -= (int)((swarmAgents[j]->direction+90-boundaryAngle-180)/(visionAngle/nodesBoundary));
	  for (int l=sensePosInit; l<=sensePosFin; l++){
	    swarmAgents[j]->states[l]=1;
	  }
	}
      }
    }

    //Lower wall
    sensePosInit=nodesAgents;
    sensePosFin= nodesAgents + nodesBoundary - 1;
    if (swarmAgents[j]->yPos<=sqrt(visionRange)){
      boundaryAngle=acos((swarmAgents[j]->yPos)/sqrt(visionRange))*180.0/cPI;
      if ((swarmAgents[j]->direction>=90-boundaryAngle+90)&&(swarmAgents[j]->direction<=90+boundaryAngle+90)){
	sensePosInit += (int)((270-boundaryAngle-swarmAgents[j]->direction+90)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
      else if ((swarmAgents[j]->direction<=boundaryAngle)||(swarmAgents[j]->direction>=360-boundaryAngle)){
	sensePosFin -= (int)((180-270-boundaryAngle+visionAngleInit)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
      else if ((swarmAgents[j]->direction>270+boundaryAngle-90)&&(swarmAgents[j]->direction<270-boundaryAngle+90)){
	sensePosInit += (int)((270-boundaryAngle-swarmAgents[j]->direction+90)/(visionAngle/nodesBoundary));
	sensePosFin -= (int)((180-270-boundaryAngle+visionAngleInit)/(visionAngle/nodesBoundary));
	for (int l=sensePosInit; l<=sensePosFin; l++){
	  swarmAgents[j]->states[l]=1;
	}
      }
    }
  }
}
  
//Function for decimal transform
int tGame::decimal(unsigned char s[]){

  int k = 0;
  for (int j=0; j<2; j++){
    k += int(s[j]&1)*pow(2, j);
  }
  return(k);
}

// wraps a position around a preset boundary (toroidal world)
void tGame::applyBoundary(double& positionVal)
{
    double val = positionVal;
    
    if (fabs(val) > boundaryDist)
    {
        if (val < 0)
        {
            val = boundaryDist - 10;
        }
        else
        {
            val = -boundaryDist + 10;
        }
    }
    
    positionVal = val;
}

/*// maintains a position within a preset boundary
 void tGame::applyBoundary(double& positionVal)
 {
     double val = positionVal;
     
     if (fabs(val) > boundaryDist)
     {
         if (val < 0)
         {
            val = -1.0 * boundaryDist;
         }
         else
         {
            val = boundaryDist;
         }
     }
     
     positionVal = val;
 }*/

/*// calculates the distance^2 between two points (toroidal world)
double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
{
    double diffX = fabs(fromX - toX);
    double diffY = fabs(fromY - toY);
    
    if (diffX > gridX)
    {
        diffX = gridXAcross - diffX;
    }
    
    if (diffY > gridY)
    {
        diffY = gridYAcross - diffY;
    }
    
    return ( diffX * diffX ) + ( diffY * diffY );
}*/

// calculates the distance^2 between two points
double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
{
    double diffX = fromX - toX;
    double diffY = fromY - toY;
    
    return ( diffX * diffX ) + ( diffY * diffY );
}

double maxFT = 0, maxST = 0, minFT = 0, minST = 0;
int ct = 0;

// calculates the angle between two agents
double tGame::calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY)
{
    double Ux = 0.0, Uy = 0.0, Vx = 0.0, Vy = 0.0;

    Ux = (toX - fromX);
    Uy = (toY - fromY);

    Vx = cos(fromAngle/180.0*cPI);
    Vy = sin(fromAngle/180.0*cPI);
    //cout<<fromAngle<<'\n';

    double firstTerm = (-(Ux * Vy) + (Uy * Vx));
    double secondTerm = ((Ux * Vx) + (Uy * Vy));
    //double theta=atan2(Uy, Ux)*180.0/cPI - fromAngle;
    //cout<<Uy<<'\t'<<Ux<<'\n';
    //cout<<firstTerm<<'\t'<<secondTerm<<'\n';

    //    return atan2(firstTerm, secondTerm) * 180.0 / cPI;
    double theta = atan2(firstTerm, secondTerm) * 180.0 / cPI;
    if (theta>=0)
      return theta;
    else
      return 360+theta;

    /*
    if (fabs(firstTerm) < 1200 && fabs(secondTerm) < 1200)
    {
        return atan2(firstTerm + 1200, secondTerm + 1200);
    }
    else
    {
        return atan2(firstTerm, secondTerm) * 180.0 / cPI;
	}*/
}

// calculates the center of the swarm and stores it in (cX, cY)
void tGame::calcSwarmCenter(double preyX[], double preyY[], bool preyDead[], double& preyCenterX, double& preyCenterY)
{
    int aliveCount = 0;
    preyCenterX = 0.0;
    preyCenterY = 0.0;
    
    for(int i = 0; i < swarmSize; ++i)
    {
        if (!preyDead[i])
        {
            preyCenterX += preyX[i];
            preyCenterY += preyY[i];
            ++aliveCount;
        }
    }
    
    preyCenterX /= (double)aliveCount;
    preyCenterY /= (double)aliveCount;
}
/*
// recalculates the prey distance lookup tables
void tGame::recalcPreyDistTable(double preyX[], double preyY[], bool preyDead[],
                                double preyToPreyDists[swarmSize][swarmSize])
{
    for (int i = 0; i < swarmSize; ++i)
    {
        if (!preyDead[i])
        {
            preyToPreyDists[i][i] = 0.0;
            
            for (int j = i + 1; j < swarmSize; ++j)
            {
                if (!preyDead[j])
                {
                    preyToPreyDists[i][j] = calcDistanceSquared(preyX[i], preyY[i], preyX[j], preyY[j]);
                    preyToPreyDists[j][i] = preyToPreyDists[i][j];
                }
            }
        }
    }
}
*/
 /*
// recalculates the prey distance lookup tables only for a given prey
void tGame::recalcPreyDistTableForOnePrey(double preyX[], double preyY[], bool preyDead[],
                                          double preyToPreyDists[swarmSize][swarmSize],
                                          int preyIndex)
{    
    for (int j = 0; j < swarmSize; ++j)
    {
        if (preyIndex != j && !preyDead[j])
        {
            preyToPreyDists[preyIndex][j] = calcDistanceSquared(preyX[preyIndex], preyY[preyIndex], preyX[j], preyY[j]);
            preyToPreyDists[j][preyIndex] = preyToPreyDists[preyIndex][j];
        }
    }
}
 */
// sums a vector of values
double tGame::sum(vector<double> values)
{
    double sum = 0.0;
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sum += *i;
    }
    
    return sum;
}

// averages a vector of values
double tGame::average(vector<double> values)
{
    return sum(values) / (double)values.size();
}

// computes the variance of a vector of values
double tGame::variance(vector<double> values)
{
    double sumSqDist = 0.0;
    double mean = average(values);
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sumSqDist += pow( *i - mean, 2.0 );
    }
    
    return sumSqDist /= (double)values.size();
}

double tGame::mutualInformation(vector<int> A,vector<int>B)
{
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]);
		nrB.insert(B[i]);
		pX[A[i]]=0.0;
		pY[B[i]]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]][B[i]]+=c;
		pX[A[i]]+=c;
		pY[B[i]]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pX[*aI]*pY[*bI]));
	return I;
	
}

double tGame::entropy(vector<int> list){
	map<int, double> p;
	map<int,double>::iterator pI;
	int i;
	double H=0.0;
	double c=1.0/(double)list.size();
	for(i=0;i<list.size();i++)
		p[list[i]]+=c;
	for (pI=p.begin();pI!=p.end();pI++) {
        H+=p[pI->first]*log2(p[pI->first]);	
	}
	return -1.0*H;
}

double tGame::ei(vector<int> A,vector<int> B,int theMask){
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]&theMask);
		nrB.insert(B[i]&theMask);
		pX[A[i]&theMask]=0.0;
		pY[B[i]&theMask]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]&theMask][B[i]&theMask]+=c;
		pX[A[i]&theMask]+=c;
		pY[B[i]&theMask]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pY[*bI]));
	return -I;
}

double tGame::computeAtomicPhi(vector<int>A,int states){
	int i;
	double P,EIsystem;
	vector<int> T0,T1;
	T0=A;
	T1=A;
	T0.erase(T0.begin()+T0.size()-1);
	T1.erase(T1.begin());
	EIsystem=ei(T0,T1,(1<<states)-1);
	P=0.0;
	for(i=0;i<states;i++){
		double EIP=ei(T0,T1,1<<i);
        //		cout<<EIP<<endl;
		P+=EIP;
	}
    //	cout<<-EIsystem+P<<" "<<EIsystem<<" "<<P<<" "<<T0.size()<<" "<<T1.size()<<endl;
	return -EIsystem+P;
}

double tGame::computeR(vector<vector<int> > table,int howFarBack){
	double Iwh,Iws,Ish,Hh,Hs,Hw,Hhws,delta,R;
	int i;
	for(i=0;i<howFarBack;i++){
		table[0].erase(table[0].begin());
		table[1].erase(table[1].begin());
		table[2].erase(table[2].begin()+(table[2].size()-1));
	}
	table[4].clear();
	for(i=0;i<table[0].size();i++){
		table[4].push_back((table[0][i]<<14)+(table[1][i]<<10)+table[2][i]);
	}
	Iwh=mutualInformation(table[0],table[2]);
    Iws=mutualInformation(table[0],table[1]);
    Ish=mutualInformation(table[1],table[2]);
    Hh=entropy(table[2]);
    Hs=entropy(table[1]);
    Hw=entropy(table[0]);
    Hhws=entropy(table[4]);
    delta=Hhws+Iwh+Iws+Ish-Hh-Hs-Hw;
    R=Iwh-delta;
  	return R;
}

double tGame::computeOldR(vector<vector<int> > table){
	double Ia,Ib;
	Ia=mutualInformation(table[0], table[2]);
	Ib=mutualInformation(table[1], table[2]);
	return Ib-Ia;
}

double tGame::predictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return mutualInformation(S, I);
}

double tGame::nonPredictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return entropy(I)-mutualInformation(S, I);
}

double tGame::predictNextInput(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	S.erase(S.begin());
	I.erase(I.begin()+I.size()-1);
	return mutualInformation(S, I);
}

void tGame::loadExperiment(char *filename){
    theExperiment.loadExperiment(filename);
}

//** tOctuplet implementation
void tOctuplet::loadOctuplet(FILE *f){
    int i,IN;
    data.clear();
    data.resize(8);
    for(i=0;i<8;i++){
        fscanf(f,"  %i",&IN);
        data[i]=IN;
    }
}

//** tEperiment class implementations
void tExperiment::loadExperiment(char *filename){
    FILE *f=fopen(filename,"r+t");
    int i,j,k;
    fscanf(f,"%i:",&j);
    dropSequences.resize(j);
    for(i=0;i<dropSequences.size();i++)
        dropSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    sizeSequences.resize(j);
    for(i=0;i<sizeSequences.size();i++)
        sizeSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    selfSequences.resize(j);
    for(i=0;i<selfSequences.size();i++)
        selfSequences[i].loadOctuplet(f);
    shouldHit.resize(drops());
    for(i=0;i<shouldHit.size();i++){
        shouldHit[i].resize(sizes());
        for(j=0;j<shouldHit[i].size();j++){
            shouldHit[i][j].resize(selves());
            for(k=0;k<shouldHit[i][j].size();k++){
                int l;
                fscanf(f,"%i\n",&l);
                if(l==1)
                    shouldHit[i][j][k]=true;
                else
                    shouldHit[i][j][k]=false;
            }
        }
    }
    fclose(f);
}

void tExperiment::showExperimentProtokoll(void){
    int i,j,k;
    printf("drop directions: %i\n",drops());
    for(i=0;i<drops();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",dropSequences[i].data[j]);
        printf("\n");
    }
    printf("drop sizes: %i\n",sizes());
    for(i=0;i<sizes();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",sizeSequences[i].data[j]);
        printf("\n");
    }
    printf("self sizes: %i\n",selves());
    for(i=0;i<selves();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",selfSequences[i].data[j]);
        printf("\n");
    }
    printf("should hit\n%i means true\nD  B   S   catch\n",(int)true);
    for(i=0;i<shouldHit.size();i++)
        for(j=0;j<shouldHit[i].size();j++)
            for(k=0;k<shouldHit[i][j].size();k++)
                printf("%i  %i  %i  %i\n",i,j,k,(int)shouldHit[i][j][k]);
}

int tExperiment::drops(void){
    return (int) dropSequences.size();
}

int tExperiment::sizes(void){
    return (int) sizeSequences.size();
}

int tExperiment::selves(void){
    return (int) selfSequences.size();
    
}
