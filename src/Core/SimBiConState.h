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

#pragma once

#include "ConUtils.h"
#include "SimGlobals.h"
#include "SimTraj.h"
#include <Core/BalanceFeedback.h>
#include <Core/Trajectory.h>
#include <MathLib/Quaternion.h>
#include <MathLib/Vector3d.h>
#include <Utils/Utils.h>

/**
 *	A simbicon controller is made up of a set of a number of states. Transition between states happen on foot contact, time out, user interaction, etc.
 *  Each controller state holds the trajectories for all the joints that are controlled. 
 */
class SimBiConState
{
    friend class ControllerEditor;
    friend class SimBiController;
    friend class CSimBiController;

protected:
    //this is the array of trajectories, one for each joint that is controlled
    DynamicArray<Trajectory *> sTraj;
    //this is a description of this state, for debugging purposes
    char description[100];
    //this is the number of the state that we should transition to in the controller's finite state machine
    int nextStateIndex;
    //this is the ammount of time that it is expected the biped will spend in this state
    double stateTime;

    //upon a transition to a new FSM state, it is assumed that the stance of the character either will be given stance, it will be reverseed , or keept the same.
    //if a state is designed for a certain stance, it is given by this variable
    //for generic states, this variable is used to determine if the stance should be reversed (as opposed to set to left or right), or stay the same.
    bool reverseStance;
    //and if this is the same, then upon entering this FSM state, the stance will remain the same
    bool keepStance;
    //if both keepStance and reverseStance are set to false, then this is the state that the character is asumed to take
    int stateStance;

    //if this variable is set to true, it indicates that the transition to the new state should occur when the swing foot contacts the ground
    //if this variable is false, then it will happen when the time of the controller goes up
    bool transitionOnFootContact;
    //if we are to allow a transition on foot contact, we need to take care of the possibility that it
    //will occur early. In this case, we may still want to switch. If phi is at least this, then it is assumed
    //that we can transition;
    double minPhiBeforeTransitionOnFootContact;
    //also, in order to make sure that we don't transition tooooo early, we expect a minimum force applied on the swing foot before
    //it should register as a contact
    double minSwingFootForceForContact;

    //this is the trajectory for the zero value of  the feedback d
    Trajectory1D *dTrajX;
    Trajectory1D *dTrajZ;

    //this is the trajectory for the zero value of  the feedback v
    Trajectory1D *vTrajX;
    Trajectory1D *vTrajZ;

public:
    /**
		default constructor
	*/
    SimBiConState(void);
    /**
		destructor
	*/
    ~SimBiConState(void);

    /**
		this method is used to determine the new stance, based on the information in this state and the old stance
	*/
    virtual int getStateStance(int oldStance);

    /**
		Returns the time we're expecting to spend in this state
	*/
    virtual double getStateTime();

    /**
		this method is used to retrieve the index of the next state 
	*/
    virtual int getNextStateIndex();

    /**
		this method is used to return the number of trajectories for this state
	*/
    virtual int getTrajectoryCount();

    /**
		Access a given trajectory
	*/
    virtual Trajectory *getTrajectory(uint idx);
    /**
		Access a given trajectory by name
	*/
    virtual Trajectory *getTrajectory(const char *name);

    /**
		This method is used to determine if, based on the parameters passed in and the type of state this is,
		the current state in the controller FSM needs to be transitioned from.
	*/
    virtual bool needTransition(double phi, double swingFootVerticalForce,
                                double stanceFootVerticalForce);

    /**
		This method makes it possible to access the state description
	*/
    virtual const char *getDescription();

    /**
		This method is used to read the state parameters from a file
	*/
    virtual void readState(FILE *f, int offset);

    /**
		This method is used to write the state parameters to a file
	*/
    virtual void writeState(FILE *f, int index);

    /** 
		Update all the trajectories to recenter them around the new given D and V trajectories
		Also save these new D and V trajectories.
	*/
    virtual void updateDVTrajectories(SimBiController *con, Joint *j,
                                      Trajectory1D &newDTrajX,
                                      Trajectory1D &newDTrajZ,
                                      Trajectory1D &newVTrajX,
                                      Trajectory1D &newVTrajZ,
                                      int nbSamples = 100);

    /**
		This method is used to read the knots of a 1D trajectory from the file, where they are specified one (knot) on a line
		The trajectory is considered complete when a line starting with endingLineType is encountered
	*/
    static void readTrajectory1D(FILE *f, Trajectory1D &result,
                                 int endingLineType);

    static void writeTrajectory1D(FILE *f, Trajectory1D &result,
                                  int startingLineType, int endingLineType);
};
