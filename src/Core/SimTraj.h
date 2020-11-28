#pragma once
#include "ConUtils.h"
#include "SimGlobals.h"
#include <Core/BalanceFeedback.h>
#include <Core/Trajectory.h>
#include <MathLib/Quaternion.h>
#include <MathLib/Vector3d.h>
#include <Utils/Utils.h>
/**
 *  This helper class is used to hold information regarding one component of a state trajectory. This includes (mainly): the base trajectory, 
 *	a data member that specifies the feedback law to be used, and the axis about which it represents a rotation, 
 */
class TrajectoryComponent
{
public:
    //this is the array of basis functions that specify the trajectories for the sagittal plane.
    Trajectory1D baseTraj;
    //if this variable is set to true, then when the stance of the character is the left side, the
    //static target provided by this trajectory should be negated
    bool reverseAngleOnLeftStance;
    //if this variable is set to true, then when the stance of the character is the right side, the
    //static target provided by this trajectory should be negated
    bool reverseAngleOnRightStance;
    //this is the rotation axis that the angles obtained from the trajectory represent rotations about
    Vector3d rotationAxis;

    //this is the balance feedback that is to be used with this trajectory
    BalanceFeedback *bFeedback;

    //this is the base value for the trajectory
    double offset;

    /**
		default constructor
	*/
    TrajectoryComponent();

    /**
		default destructor.
	*/
    ~TrajectoryComponent();

    /**
		this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
		and the d and v values used for feedback.
	*/
    virtual Quaternion evaluateTrajectoryComponent(SimBiController *con,
                                                   Joint *j, int stance,
                                                   double phi,
                                                   const Vector3d &d,
                                                   const Vector3d &v);

    /**
		this method is used to evaluate the feedback contribution, given the current phase, d and v.
	*/
    virtual double computeFeedback(SimBiController *con, Joint *j, double phi,
                                   const Vector3d &d, const Vector3d &v);
    /** 
		Update this component to recenter it around the new given D and V trajectories
	*/
    virtual void
    updateComponent(SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
                    Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX,
                    Trajectory1D &newVTrajZ, Trajectory1D *oldDTrajX,
                    Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
                    Trajectory1D *oldVTrajZ, int nbSamples);

    /**
		This method is used to read the knots of a base trajectory from the file, where they are specified one (knot) on a line
	*/
    // virtual void readBaseTrajectory(FILE *f);

    /**
		This method is used to read a trajectory from a file
	*/
    virtual void readTrajectoryComponent(FILE *f);

    /**
		This method is used to read a trajectory from a file
	*/
    virtual void writeBaseTrajectory(FILE *f);

    /**
		This method is used to write a trajectory to a file
	*/
    virtual void writeTrajectoryComponent(FILE *f);
};

// #define ENABLE_SIMBICON_EDITOR
/**
 *  This helper class is used to hold information regarding one state trajectory. This includes: a sequence of components, 
 *	the index of the joint that this trajectory applies to, the coordinate frame in which the final orientation is expressed, etc.
 */
class Trajectory
{
protected:
#ifndef ENABLE_SIMBICON_EDITOR
    //these are the components that define the current trajectory
    DynamicArray<TrajectoryComponent *> components;
#endif
public:
//if the biped that is controlled is in a left-sideed stance, then this is the index of the joint that
//the trajectory is used to control - it is assumed that if this is -1, then the trajectory applies
//to the torso, and not to a joint
#ifdef ENABLE_SIMBICON_EDITOR
    //these are the components that define the current trajectory
    DynamicArray<TrajectoryComponent *> components;
#endif
    int leftStanceIndex;
    //and this is the index of the joint that the trajectory is associated to if the biped is in a
    //right-side stance
    int rightStanceIndex;
    //we'll keep the joint name, for debugging purposes
    char jName[100];

    //if this variable is set to true, then the desired orientation here is expressed in character coordinates, otherwise it is relative
    //to the parent
    bool relToCharFrame;

    //this is the trajectory for the strength of the joing.
    Trajectory1D *strengthTraj;

    /**
		default constructor
	*/
    Trajectory();

    /**
		default destructor.
	*/
    virtual ~Trajectory();

    /**
		this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
		and the d and v values used for feedback.
	*/
    virtual Quaternion evaluateTrajectory(SimBiController *con, Joint *j,
                                          int stance, double phi,
                                          const Vector3d &d, const Vector3d &v);

    /**
		this method is used to evaluate the strength of the joint at a given value of phi.
	*/
    virtual double evaluateStrength(double phiToUse);

    /**
		this method returns the joint index that this trajectory applies to, unless this applies to the root, in which case it returns -1.
	*/
    virtual int getJointIndex(int stance);

    /** 
		Update all the components to recenter them around the new given D and V trajectories
	*/
    virtual void
    updateComponents(SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
                     Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX,
                     Trajectory1D &newVTrajZ, Trajectory1D *oldDTrajX,
                     Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
                     Trajectory1D *oldVTrajZ, int nbSamples);

    /**
		This method is used to read a trajectory from a file
	*/
    virtual void readTrajectory(FILE *f);

    /**
		This method is used to write the knots of a strength trajectory to the file, where they are specified one (knot) on a line
	*/
    virtual void writeStrengthTrajectory(FILE *f);

    /**
		This method is used to write a trajectory to a file
	*/
    virtual void writeTrajectory(FILE *f);
};
