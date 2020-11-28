/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "SimBiConState.h"
#include "CustomSimTraj.h"
#include "SimBiController.h"
#include "SimGlobals.h"
#include <Utils/Utils.h>
using namespace std;
/**
		default constructor
	*/
SimBiConState::SimBiConState(void)
{
    strcpy(description, "Uninitialized state");
    nextStateIndex = -1;
    this->stateTime = 0;
    transitionOnFootContact = true;
    minPhiBeforeTransitionOnFootContact = 0.5;
    minSwingFootForceForContact = 20.0;
    reverseStance = false;
    keepStance = false;

    dTrajX = NULL;
    dTrajZ = NULL;
    vTrajX = NULL;
    vTrajZ = NULL;
}

/**
    destructor
*/
SimBiConState::~SimBiConState(void)
{
    for (uint i = 0; i < sTraj.size(); i++)
        delete sTraj[i];
}

/**
    this method is used to determine the new stance, based on the information in this state and the old stance
*/
int SimBiConState::getStateStance(int oldStance)
{
    if (keepStance == true)
        return oldStance;
    if (reverseStance == false)
        return stateStance;
    if (oldStance == LEFT_STANCE)
        return RIGHT_STANCE;
    return LEFT_STANCE;
}

/**
    Returns the time we're expecting to spend in this state
*/
double SimBiConState::getStateTime() { return stateTime; }

/**
    this method is used to retrieve the index of the next state 
*/
int SimBiConState::getNextStateIndex() { return nextStateIndex; }

/**
    this method is used to return the number of trajectories for this state
*/
int SimBiConState::getTrajectoryCount() { return sTraj.size(); }

/**
    Access a given trajectory
*/
Trajectory *SimBiConState::getTrajectory(uint idx)
{
    if (idx >= sTraj.size())
        return NULL;
    return sTraj[idx];
}

/**
    Access a given trajectory by name
*/
Trajectory *SimBiConState::getTrajectory(const char *name)
{
    for (uint i = 0; i < sTraj.size(); i++)
        if (strcmp(sTraj[i]->jName, name) == 0)
            return sTraj[i];
    return NULL;
}

/**
    This method is used to determine if, based on the parameters passed in and the type of state this is,
    the current state in the controller FSM needs to be transitioned from.
*/
bool SimBiConState::needTransition(double phi, double swingFootVerticalForce,
                                   double stanceFootVerticalForce)
{
    //if it is a foot contact based transition
    if (transitionOnFootContact == true)
    {
        //transition if we have a meaningful foot contact, and if it does not happen too early on...
        if ((phi > minPhiBeforeTransitionOnFootContact &&
             swingFootVerticalForce > minSwingFootForceForContact) ||
            phi >= 1)
            return true;
        return false;
    }

    //otherwise it must be a time-based transition
    if (phi >= 1)
        return true;

    return false;
}

/**
    This method makes it possible to access the state description
*/
const char *SimBiConState::getDescription() { return description; }

/**
	This method is used to read the state parameters from a file
*/
void SimBiConState::readState(FILE *f, int offset)
{
    if (f == NULL)
        throwError("File pointer is NULL - cannot read gain coefficients!!");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    Trajectory *tempTraj;

    //this is where it happens.
    while (!feof(f))
    {
        //get a line from the file...
        fgets(buffer, 200, f);
        if (strlen(buffer) > 195)
            throwError("The input file contains a line that is longer than "
                       "~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getConLineType(line);
        switch (lineType)
        {
        case CON_STATE_END:
            //we're done...
            return;
            break;
        case CON_CUSTOM_STATE_END:
            return;
            break;
        case CON_NEXT_STATE:
            if (sscanf(line, "%d", &this->nextStateIndex) != 1)
                throwError("An index must be specified when using the "
                           "\'nextState\' keyword");
            this->nextStateIndex += offset;
            break;
        case CON_STATE_DESCRIPTION:
            strcpy(this->description, trim(line));
            break;
        case CON_STATE_TIME:
            if (sscanf(line, "%lf", &stateTime) != 1)
                throwError("The time that is expected to be spent in this "
                           "state needs to be provided.");
            break;
        case CON_STATE_STANCE:
            reverseStance = false;
            keepStance = false;
            if (strncmp(trim(line), "left", 4) == 0)
                stateStance = LEFT_STANCE;
            else if (strncmp(trim(line), "right", 5) == 0)
                stateStance = RIGHT_STANCE;
            else if (strncmp(trim(line), "reverse", 7) == 0)
                reverseStance = true;
            else if (strncmp(trim(line), "same", 4) == 0)
                keepStance = true;
            else
                throwError("When using the \'stateStance\' keyword, \'left\', "
                           "\'right\' or \'reverse\' must be specified.");
            break;
        case CON_TRANSITION_ON:
            transitionOnFootContact = false;
            if (strncmp(trim(line), "footDown", 8) == 0)
                transitionOnFootContact = true;
            else if (strncmp(trim(line), "timeUp", 6) == 0)
                //nothn' to do, since this is the default
                ;
            else
                throwError("When using the \'transitionOn\' keyword, "
                           "\'footDown\' or \'timeUp\' must be specified.");
            break;
        case CON_TRAJECTORY_START:
            //create a new trajectory, and read its information from the file
            tempTraj = new Trajectory();
            strcpy(tempTraj->jName, trim(line));
            printf("[debug] read joint %s trajectory\n", tempTraj->jName);
            tempTraj->readTrajectory(f);
            sTraj.push_back(tempTraj);
            break;

        case CON_CUSTOM_TRAJECTORY_START:
            tempTraj = new CustomTrajectory();
            strcpy(tempTraj->jName, trim(line));
            printf("[debug] read joint %s custom trajectory\n",
                   tempTraj->jName);
            tempTraj->readTrajectory(f);
            sTraj.push_back(tempTraj);
            break;
        case CON_D_TRAJX_START:
            if (dTrajX != NULL)
                throwError("Two dTrajX trajectory, this is illegal!");
            dTrajX = new Trajectory1D();
            readTrajectory1D(f, *dTrajX, CON_D_TRAJX_END);
            break;
        case CON_D_TRAJZ_START:
            if (dTrajZ != NULL)
                throwError("Two dTrajZ trajectory, this is illegal!");
            dTrajZ = new Trajectory1D();
            readTrajectory1D(f, *dTrajZ, CON_D_TRAJZ_END);
            break;

        case CON_V_TRAJX_START:
            if (vTrajX != NULL)
                throwError("Two vTrajX trajectory, this is illegal!");
            vTrajX = new Trajectory1D();
            readTrajectory1D(f, *vTrajX, CON_V_TRAJX_END);
            break;

        case CON_V_TRAJZ_START:
            if (vTrajZ != NULL)
                throwError("Two vTrajZ trajectory, this is illegal!");
            vTrajZ = new Trajectory1D();
            readTrajectory1D(f, *vTrajZ, CON_V_TRAJZ_END);
            break;

        case CON_COMMENT:
            break;

        case CON_NOT_IMPORTANT:
            tprintf("Ignoring input line: \'%s\'\n", line);
            break;

        default:
            throwError(
                "Incorrect SIMBICON input file: \'%s\' - unexpected line.",
                buffer);
        }
    }
    throwError("Incorrect SIMBICON input file: No \'/State\' found", buffer);
}

/**
	This method is used to write the state parameters to a file
*/
void SimBiConState::writeState(FILE *f, int index)
{
    if (f == NULL)
        return;

    fprintf(f, "%s %d\n", getConLineString(CON_STATE_START), index);

    fprintf(f, "\t%s %s\n", getConLineString(CON_STATE_DESCRIPTION),
            description);
    fprintf(f, "\t%s %d\n", getConLineString(CON_NEXT_STATE), nextStateIndex);
    fprintf(f, "\t%s %s\n", getConLineString(CON_TRANSITION_ON),
            transitionOnFootContact ? "footDown" : "timeUp");

    if (reverseStance)
        fprintf(f, "\t%s reverse\n", getConLineString(CON_STATE_STANCE));
    else if (keepStance)
        fprintf(f, "\t%s same\n", getConLineString(CON_STATE_STANCE));
    else if (stateStance == LEFT_STANCE)
        fprintf(f, "\t%s left\n", getConLineString(CON_STATE_STANCE));
    else if (stateStance == RIGHT_STANCE)
        fprintf(f, "\t%s right\n", getConLineString(CON_STATE_STANCE));

    fprintf(f, "\t%s %lf\n", getConLineString(CON_STATE_TIME), stateTime);

    fprintf(f, "\n");

    if (dTrajX != NULL)
        writeTrajectory1D(f, *dTrajX, CON_D_TRAJX_START, CON_D_TRAJX_END);
    if (dTrajZ != NULL)
        writeTrajectory1D(f, *dTrajZ, CON_D_TRAJZ_START, CON_D_TRAJZ_END);
    if (vTrajX != NULL)
        writeTrajectory1D(f, *vTrajX, CON_V_TRAJX_START, CON_V_TRAJX_END);
    if (vTrajZ != NULL)
        writeTrajectory1D(f, *vTrajZ, CON_V_TRAJZ_START, CON_V_TRAJZ_END);

    fprintf(f, "\n");

    for (uint i = 0; i < sTraj.size(); ++i)
    {
        fprintf(f, "\n");
        sTraj[i]->writeTrajectory(f);
    }

    fprintf(f, "%s\n", getConLineString(CON_STATE_END));
}

/**
	This method is used to read the knots of a strength trajectory from the file, where they are specified one (knot) on a line
*/
void SimBiConState::readTrajectory1D(FILE *f, Trajectory1D &result,
                                     int endingLineType)
{
    if (f == NULL)
        throwError("File pointer is NULL - cannot read gain coefficients!!");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    double temp1, temp2;

    //this is where it happens.
    while (!feof(f))
    {
        //get a line from the file...
        fgets(buffer, 200, f);
        if (strlen(buffer) > 195)
            throwError("The input file contains a line that is longer than "
                       "~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getConLineType(line);
        if (lineType == endingLineType)
            //we're done...
            return;

        switch (lineType)
        {
        case CON_COMMENT:
            break;
        case CON_NOT_IMPORTANT:
            //we expect pairs of numbers, one pair on each row, so see if we have a valid pair
            if (sscanf(line, "%lf %lf", &temp1, &temp2) == 2)
            {
                result.addKnot(temp1, temp2);
            }
            else
                tprintf("Ignoring input line: \'%s\'\n", line);
            break;
        default:
            throwError(
                "Incorrect SIMBICON input file: \'%s\' - unexpected line.",
                buffer);
        }
    }
    throwError("Incorrect SIMBICON input file: Trajectory not closed ", buffer);
}

/**
	This method is used to write a trajectory to the file
*/
void SimBiConState::writeTrajectory1D(FILE *f, Trajectory1D &result,
                                      int startingLineType, int endingLineType)
{
    if (f == NULL)
        return;

    fprintf(f, "\t%s\n", getConLineString(startingLineType));

    for (int i = 0; i < result.getKnotCount(); ++i)
    {
        fprintf(f, "\t\t%lf %lf\n", result.getKnotPosition(i),
                result.getKnotValue(i));
    }

    fprintf(f, "\t%s\n", getConLineString(endingLineType));
}

/** 
	Update all the trajectories to recenter them around the new given D and V trajectories
	Also save these new D and V trajectories.
*/
void SimBiConState::updateDVTrajectories(SimBiController *con, Joint *j,
                                         Trajectory1D &newDTrajX,
                                         Trajectory1D &newDTrajZ,
                                         Trajectory1D &newVTrajX,
                                         Trajectory1D &newVTrajZ, int nbSamples)
{

    int nbTraj = sTraj.size();
    for (int i = 0; i < nbTraj; ++i)
    {
        sTraj[i]->updateComponents(con, j, newDTrajX, newDTrajZ, newVTrajX,
                                   newVTrajZ, dTrajX, dTrajZ, vTrajX, vTrajZ,
                                   nbSamples);
    }

    if (dTrajX != NULL)
        delete dTrajX;
    if (dTrajZ != NULL)
        delete dTrajZ;
    if (vTrajX != NULL)
        delete vTrajX;
    if (vTrajZ != NULL)
        delete vTrajZ;
    dTrajX = new Trajectory1D(newDTrajX);
    dTrajZ = new Trajectory1D(newDTrajZ);
    vTrajX = new Trajectory1D(newVTrajX);
    vTrajZ = new Trajectory1D(newVTrajZ);
}
