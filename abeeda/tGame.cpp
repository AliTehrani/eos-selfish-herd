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
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>
//#include <iostream.h>
//#include <time.h>
//#include <fstream.h>


#define cPI 3.14159265

// simulation-specific constants
#define preyVisionRange         200.0 * 200.0
#define preyVisionAngle         360.0 / 2.0
#define preySensors             24
#define totalStepsInSimulation  1000
#define gridX                   384.0
#define gridY                   384.0
#define gridXAcross             2.0 * gridX
#define gridYAcross             2.0 * gridY
#define collisionDist           5.0 * 5.0
#define boundaryDist            gridX - sqrt(collisionDist)
#define stepsUntilPredation     250

#define swarmSize               100
#define V                       0.2
#define dTheta                  6.0
#define dTime                   0.05
#define penalty					1.0

// precalculated lookup tables for the game
double cosLookup[360];
double sinLookup[360];
double atan2Lookup[2400][2400];

tGame::tGame()
{
    // fill lookup tables
    for (int i = 0; i < 360; ++i)
    {
        cosLookup[i] = cos((double)i * (cPI / 180.0));
        sinLookup[i] = sin((double)i * (cPI / 180.0));
    }
    
    for (int i = 0; i < 2400; ++i)
    {
        for (int j = 0; j < 2400; ++j)
        {
            atan2Lookup[i][j] = atan2(i - 1200, j - 1200) * 180.0 / cPI;
        }
    }
}

tGame::~tGame() { }

// runs the simulation for the given agent(s)
string tGame::executeGame(vector<tAgent*> swarmAgents, FILE *data_file, bool report, bool collision, double startingDist, int killDelay)
{
  string reportString = "";

  // set up the prey agents from swarmAgents (above)
  //FYI swarmAgents[0]->fitness;
  
  // loop the following for x number of updates

  // fill in the Markov Network sensors for each swarmAgent

  // execute updateStates() for each swarmAgent

  // get outputs from each swarmAgent and have that encode some action
  //swarmAgents[i]->states[maxNodes - 1];
  //swarmAgents[i]->states[maxNodes - 2];

  // evaluate fitness for that update
  // if they collide, lose fitness
  // if they move without collision, gain small amount of fitness
  
  // make sure each swarmAgent has its fitness assigned
  //FYI  swarmAgents[i]->fitness = some value;

    enum Action {move=0, stay, left, right};
    Action action;
    //    const int height=1, width=1;
    //    const flyDiam=0.01, V=0.3, rSight=0.1, thetaSight=pi, dTime=0.1, dTheta=6*pi/180.0;
    //    double distance, penalty=member[1].fitness/double(nIteration);

//    ofstream trajectory("trajectory.txt");

    //    int st[2];
    int i, j;
    int repeat;
    //cout<<(rand()%int(width/flyDiam))*flyDiam<<endl;
//  Initial placement
    for (j=1; j<=swarmSize; j++){
      swarmAgents[j]->xPos=randDouble*(gridX-2*collisionDist)+collisionDist;//      member[j].x[0]=(rand()%int(width/flyDiam))*flyDiam;
      swarmAgents[j]->yPos=randDouble*gridY;//      member[j].y[0]=(rand()%int(height/flyDiam))*flyDiam;
      //      member[j].fitness=nSwarm;
//   Check if positions overlap
      do{
	repeat=0;
	for (int m=1; m<=j-1; m++){
	  //double
	  if (sqrt(pow(swarmAgents[j]->xPos-swarmAgents[m]->xPos, 2)+pow(swarmAgents[j]->yPos-swarmAgents[m]->yPos, 2))<collisionDist){
	    repeat=1;
	    break;
	  }
	}
      }while(repeat==1);

      swarmAgents[j]->direction=(rand()%60)*6*cPI/180.0;
      //cout<<member[j].x[0]<<'\t'<<member[j].y[0]<<'\t'<<member[j].theta<<endl;
    }

//  Iterating...
    for (i=1; i<=totalStepsInSimulation; i++){
      for (j=1; j<=swarmSize; j++){
	//	st[0]=rand()%2;
	//	st[1]=rand()%2;
	//cout<<st[1]<<'\t'<<st[0]<<endl;
	//	action=static_cast<Action>(decimal(st));
    //    action=static_cast<Action>(decimal(swarmAgents[i]->states));
	//cout<<i<<'\t'<<action<<endl;
	switch (((swarmAgents[i]->states[maxNodes-2]>>1)&1)
			+(swarmAgents[i]->states[maxNodes-1]&1)
			){
	case move:
	  {
	    swarmAgents[j]->xPos = swarmAgents[j]->xPos+V*cos(swarmAgents[j]->direction)*dTime;
	    swarmAgents[j]->yPos = swarmAgents[j]->yPos+V*sin(swarmAgents[j]->direction)*dTime;
	  }
	case stay:
	  {
            swarmAgents[j]->xPos = swarmAgents[j]->xPos+V*cos(swarmAgents[j]->direction)*dTime;
            swarmAgents[j]->yPos = swarmAgents[j]->yPos+V*sin(swarmAgents[j]->direction)*dTime;
	    //	    member[j].x[i]=member[j].x[i-1];
	    //	    member[j].y[i]=member[j].y[i-1];
	  }
	case left:
	  {
            swarmAgents[j]->direction +=dTheta;  
            swarmAgents[j]->xPos = swarmAgents[j]->xPos+V*cos(swarmAgents[j]->direction)*dTime;
            swarmAgents[j]->yPos = swarmAgents[j]->yPos+V*sin(swarmAgents[j]->direction)*dTime;
	    //	    member[j].theta +=dTheta;
	    //	    member[j].x[i] = member[j].x[i-1]+V*cos(member[j].theta)*dTime;
	    //	    member[j].y[i] = member[j].y[i-1]+V*sin(member[j].theta)*dTime;
	  }
	case right:
	  {
	    swarmAgents[j]->direction -=dTheta;
	    swarmAgents[j]->xPos = swarmAgents[j]->xPos+V*cos(swarmAgents[j]->direction)*dTime;
	    swarmAgents[j]->yPos = swarmAgents[j]->yPos+V*sin(swarmAgents[j]->direction)*dTime;
	  }
	}
	//Domain boundaries
	if (swarmAgents[j]->xPos-collisionDist/2.0>=gridX){
	  swarmAgents[j]->xPos= 2*gridX-swarmAgents[j]->xPos;
	  swarmAgents[j]->direction=cPI-swarmAgents[j]->direction;

	}
	if (swarmAgents[j]->yPos-collisionDist/2.0>=gridY){
	  swarmAgents[j]->yPos= 2*gridY-swarmAgents[j]->yPos;
	  swarmAgents[j]->direction=-swarmAgents[j]->direction;
	}
	if (swarmAgents[j]->xPos<=0){
	  swarmAgents[j]->xPos= -swarmAgents[j]->xPos;
	  swarmAgents[j]->direction=cPI-swarmAgents[j]->direction;
	}
	if (swarmAgents[j]->yPos<=0){
	  swarmAgents[j]->yPos=-swarmAgents[j]->yPos;
	  swarmAgents[j]->direction=-swarmAgents[j]->direction;
	}

	//if (j==1)
	//cout<<member[j].x[i]<<'\t'<<member[j].y[i]<<'\n';
      }
      //Collisions
      for (int m=1; m<=swarmSize-1; m++){
	for (int n=m+1; n<=swarmSize; n++){
		if (sqrt(pow(swarmAgents[m]->xPos-swarmAgents[n]->xPos, 2)+pow(swarmAgents[m]->yPos-swarmAgents[n]->yPos, 2))<=collisionDist){
	    swarmAgents[m]->fitness -= penalty;
	    swarmAgents[n]->fitness -= penalty;
	    //cout<<member[m].y[i]<<'\t'<<member[n].y[i]<<'\t'<<distance<<endl;
	  }
	}
      }

    }
    /*
    //Writing to file
    for (j=1; j<=nSwarm; j++){
      trajectory<<"zone"<<endl;
      cout<<member[j].fitness<<endl;
      for (i=0; i<=nIteration; i++){
	trajectory<<member[j].x[i]<<'\t'<<member[j].y[i]<<'\n';
      }
    }
    */
  return reportString;
}

//Function for decimal transform
int tGame::decimal(unsigned char s[]){

  int k = 0;
  for (int j=0; j<=1; j++){
    //cout<<gen[j];
    k += int(s[j]*pow(2, j));
  }
  //cout<<"   "<<k<<endl;
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
    
    Vx = cosLookup[(int)fromAngle];
    Vy = sinLookup[(int)fromAngle];
    
    int firstTerm = (int)((Ux * Vy) - (Uy * Vx));
    int secondTerm = (int)((Ux * Vx) + (Uy * Vy));
    
    if (fabs(firstTerm) < 1200 && fabs(secondTerm) < 1200)
    {
        return atan2Lookup[firstTerm + 1200][secondTerm + 1200];
    }
    else
    {
        return atan2(firstTerm, secondTerm) * 180.0 / cPI;
    }
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

int tGame::neuronsConnectedToPreyRetina(tAgent *agent){
    tAgent *A=new tAgent;
    int i,j,c=0;
    A->genome=agent->genome;
    A->setupPhenotype();
    for(i=0;i<A->hmmus.size();i++)
        for(j=0;j<A->hmmus[i]->ins.size();j++)
            if(A->hmmus[i]->ins[j]<preySensors)
                c++;
    delete A;
    return c;
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
