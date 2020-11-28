#include "SimBiConState.h"

/**
 * \brief           A Custom simbicon state which is simpler
 * 1. the trajectory is discrete and keeps the same in a period of time
 * 2. there is no d and v trajectory
*/
namespace Json
{
class Value;
}
class CustomSimBiConState : public SimBiConState
{
public:
    CustomSimBiConState();

    virtual int getStateStance(int oldStance) override;

    virtual double getStateTime() override;

    virtual int getStateIndex();
    virtual int getNextStateIndex() override;

    virtual int getTrajectoryCount() override;

    virtual Trajectory *getTrajectory(uint idx) override;

    virtual Trajectory *getTrajectory(const char *name) override;

    virtual bool needTransition(double phi, double swingFootVerticalForce,
                                double stanceFootVerticalForce) override;

    virtual const char *getDescription() override;

    virtual void readState(FILE *f, int offset) override;
    virtual void readState(const Json::Value &value, int offset);

    virtual void writeState(FILE *f, int index) override;

    virtual void updateDVTrajectories(SimBiController *con, Joint *j,
                                      Trajectory1D &newDTrajX,
                                      Trajectory1D &newDTrajZ,
                                      Trajectory1D &newVTrajX,
                                      Trajectory1D &newVTrajZ,
                                      int nbSamples /*= 100*/) override;

protected:
    int myStateId;

    // read from the custom state
    virtual void buildTrajectories(const Json::Value &value);
};