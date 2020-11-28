#pragma once
#include "SimTraj.h"

namespace Json
{
class Value;
};
/**
 * \brief               Custom trajectory component, evaluate the target trajectory discretely
*/
class CustomTrajectoyComponent : public TrajectoryComponent
{
public:
    CustomTrajectoyComponent();
    virtual ~CustomTrajectoyComponent();

    virtual Quaternion evaluateTrajectoryComponent(SimBiController *con,
                                                   Joint *j, int stance,
                                                   double phi,
                                                   const Vector3d &d,
                                                   const Vector3d &v) override;

    virtual double computeFeedback(SimBiController *con, Joint *j, double phi,
                                   const Vector3d &d,
                                   const Vector3d &v) override;
    virtual void
    updateComponent(SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
                    Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX,
                    Trajectory1D &newVTrajZ, Trajectory1D *oldDTrajX,
                    Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
                    Trajectory1D *oldVTrajZ, int nbSamples) override;

    // virtual void readBaseTrajectory(FILE *f) override;

    virtual void readTrajectoryComponent(FILE *f) override;
    virtual void readTrajectoryComponent(const Json::Value &tar_quater);

    virtual void writeBaseTrajectory(FILE *f) override;

    virtual void writeTrajectoryComponent(FILE *f) override;

protected:
    double getNextKnotValue(const double phi, int num_of_interval);
    Quaternion computeCustomFeedback(SimBiController *con, Joint *j, double phi,
                                     const Vector3d &d, const Vector3d &v);
};

class CustomTrajectory : public Trajectory
{
public:
    CustomTrajectory();
    virtual ~CustomTrajectory();

    virtual Quaternion evaluateTrajectory(SimBiController *con, Joint *j,
                                          int stance, double phi,
                                          const Vector3d &d,
                                          const Vector3d &v) override;
    virtual double evaluateStrength(double phiToUse) override;
    virtual int getJointIndex(int stance) override;
    virtual void
    updateComponents(SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
                     Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX,
                     Trajectory1D &newVTrajZ, Trajectory1D *oldDTrajX,
                     Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
                     Trajectory1D *oldVTrajZ, int nbSamples) override;
    virtual void readTrajectory(FILE *f) override;
    virtual void readTrajectory(const Json::Value &value);
    virtual void writeStrengthTrajectory(FILE *f) override;
    virtual void writeTrajectory(FILE *f) override;
};