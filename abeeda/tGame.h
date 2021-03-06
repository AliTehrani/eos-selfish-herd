/*
 * tGame.h
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
 
#ifndef _tGame_h_included_
#define _tGame_h_included_

#include "globalConst.h"
#include "tAgent.h"
#include <vector>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <sstream>
//#include <iostream.h>
//#include <time.h>
using namespace std;

class tOctuplet{
public:
    vector<int> data;
    void loadOctuplet(FILE *f);
};

class tExperiment{
public:
    vector<tOctuplet> dropSequences,sizeSequences,selfSequences;
    vector<vector<vector<bool> > > shouldHit;
    void loadExperiment(char *filename);
    void showExperimentProtokoll(void);
    int drops(void);
    int sizes(void);
    int selves(void);
};

class tGame{
public:
    int decimal(unsigned char s[]);
    tExperiment theExperiment;
    void loadExperiment(char *filename);
    string executeGame(vector<tAgent*> swarmAgents, FILE *data_file, bool &report, long &numberCollision, double &avgSwarmFitness, int killDelay, int deme);
    tGame();
    ~tGame();
    void applyBoundary(double& positionVal);
    void initialPlacement(vector<tAgent*> swarmAgents);
    void printToString(vector<tAgent*> swarmAgents, int& step, ostringstream& sstm);
    void wallsCheck(double xTemp, double yTemp, double &xPos, double &yPos, double &direction, double &fitness);   
    void wallsCheck_2(double xTemp, double yTemp, double &xPos, double &yPos, double &direction, double &fitness);
    void senseStates(vector<tAgent*> swarmAgents);
    double calcDistanceSquared(double fromX, double fromY, double toX, double toY);
    double calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY);
    void bounceBack(double &x1, double &y1, double &theta1, double x2, double y2);
    void noOverlap(double &xPos, double &yPos, double direction);
    void calcSwarmCenter(double preyX[], double preyY[], bool preyDead[], double& preyCenterX, double& preyCenterY);
    //    void recalcPreyDistTable(double preyX[], double preyY[], bool preyDead[],
    //                             double preyToPreyDists[swarmSize][swarmSize]);
//    void recalcPreyDistTableForOnePrey(double preyX[], double preyY[], bool preyDead[],
//                                       double preyToPreyDists[swarmSize][swarmSize],
    //                                     int preyIndex);
    double sum(vector<double> values);
    double average(vector<double> values);
    double variance(vector<double> values);
    double mutualInformation(vector<int> A,vector<int>B);
    double ei(vector<int> A,vector<int> B,int theMask);
    double computeAtomicPhi(vector<int>A,int states);
    double predictiveI(vector<int>A);
    double nonPredictiveI(vector<int>A);
    double predictNextInput(vector<int>A);
    double computeR(vector<vector<int> > table,int howFarBack);
    double computeOldR(vector<vector<int> > table);
    double entropy(vector<int> list);
};
#endif
