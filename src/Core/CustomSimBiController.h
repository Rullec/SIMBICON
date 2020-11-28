#include "SimBiController.h"

class CSimBiController : public SimBiController
{
    friend class ConCompositionFramework;
    friend class SimbiconPlayer;
    friend class ControllerEditor;

public:
    CSimBiController(Character *b);
    virtual ~CSimBiController(void);

    virtual void computeTorques(DynamicArray<ContactPoint> *cfs);

    virtual int advanceInTime(double dt, DynamicArray<ContactPoint> *cfs);

    virtual void getControllerState(SimBiControllerState *cs);

    virtual void setControllerState(const SimBiControllerState &cs);

    virtual void loadFromFile(char *fName);

    virtual bool isBodyInContactWithTheGround();

    virtual double getPhase();

    virtual Point3d getStanceFootPos();

    virtual Point3d getSwingFootPos();

    virtual void writeToFile(char *fileName, char *stateFileName = NULL);

    virtual int getFSMState();

    virtual Quaternion getCharacterFrame();

    virtual void updateDAndV();

    virtual SimBiConState *getState(uint idx);

    virtual void updateTrackingPose(DynamicArray<double> &trackingPose,
                                    double phiToUse = -1);

    virtual int getStance();

    virtual void computeD0(double phi, Vector3d *d0);

    virtual void computeV0(double phi, Vector3d *v0);

protected:
    virtual void parseGainLine(char *line);

    virtual void setFSMStateTo(int index);

    virtual void transitionToState(int stateIndex);

    virtual Vector3d getForceOn(RigidBody *rb, DynamicArray<ContactPoint> *cfs);

    virtual Vector3d getForceOnFoot(RigidBody *foot,
                                    DynamicArray<ContactPoint> *cfs);

    virtual bool isFoot(RigidBody *rb);

    virtual bool isStanceFoot(RigidBody *rb);

    virtual bool isSwingFoot(RigidBody *rb);

    virtual double getStanceFootWeightRatio(DynamicArray<ContactPoint> *cfs);

    virtual void computeHipTorques(const Quaternion &qRootD,
                                   const Quaternion &qSwingHipD,
                                   double stanceHipToSwingHipRatio);

    virtual void resolveJoints(SimBiConState *state);

    virtual void setStance(int newStance);

    virtual RigidBody *getRBBySymbolicName(char *sName);

    virtual Quaternion getTargetPose(ReducedCharacterState *rc_state);

    virtual void loadCustomFSMFromFile(const std::string &file);
    double curTime;
};
