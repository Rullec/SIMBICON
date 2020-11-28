#include "CustomSimBiController.h"
#include "CustomSimBiConState.h"
#include "SimBiConState.h"
#include "SimTraj.h"
#include "Utils/JsonUtil.h"
#include "Utils/LogUtil.h"

CSimBiController::CSimBiController(Character *b) : SimBiController(b)
{
    curTime = 0;
}
CSimBiController::~CSimBiController(void) {}

/**
 * \brief       Compute the simbicon control torques
 * \param cfs   contact points
*/
void CSimBiController::computeTorques(DynamicArray<ContactPoint> *cfs)
{
    std::cout << "------------compute torques begin------------\n";
    std::cout << "[csbc] stance: "
              << ((stance == LEFT_STANCE) ? ("left") : ("right")) << " leg\n";
    std::cout << "[csbc] cur state id " << this->FSMStateIndex << std::endl;
    // 1. get the new target pose
    CON_ASSERT(FSMStateIndex <= (int)states.size());

    ReducedCharacterState poseRS(&desiredPose);
    // 1.1 update d and v
    updateDAndV();

    // 1.2 do balance policy to adjust target pose
    Quaternion root_target = getTargetPose(&poseRS);

    // 2. calculate the individual PD control force
    PoseController::computeTorques(cfs);

    // 3. change the stance hip control force by torso virtual PD force and swing hip force
    // 3.1 calculate the stance foot force / total foot force
    double stance_swing_ratio = getStanceFootWeightRatio(cfs);

    // 3.2 given root targetpose and swing/stance ratio, calculate the torques for swing & stance hip
    computeHipTorques(root_target,
                      poseRS.getJointRelativeOrientation(swingHipIndex),
                      stance_swing_ratio);
}

int CSimBiController::advanceInTime(double dt, DynamicArray<ContactPoint> *cfs)
{

    std::cout << "--------simbicon advance, cur time " << curTime
              << " dt = " << dt << std::endl;
    curTime += dt;
    return SimBiController::advanceInTime(dt, cfs);
}

void CSimBiController::getControllerState(SimBiControllerState *cs)
{
    SimBiController::getControllerState(cs);
}

void CSimBiController::setControllerState(const SimBiControllerState &cs)
{
    SimBiController::setControllerState(cs);
}

void CSimBiController::loadFromFile(char *fName)
{
    // 1. load old file
    SimBiController::loadFromFile(fName);

    // 2. load myself's config file (FSM)
#ifdef USE_CUSTOM_FSM
    std::string fsm_path = "../data/controllers/xudong/fsm_custom.json";
    loadCustomFSMFromFile(fsm_path);
    CON_INFO("load custom fsm from {}", fsm_path);
#endif
}

bool CSimBiController::isBodyInContactWithTheGround()
{
    return SimBiController::isBodyInContactWithTheGround();
}

double CSimBiController::getPhase() { return SimBiController::getPhase(); }

Point3d CSimBiController::getStanceFootPos()
{
    return SimBiController::getStanceFootPos();
}

Point3d CSimBiController::getSwingFootPos()
{
    return SimBiController::getSwingFootPos();
}

void CSimBiController::writeToFile(char *fileName,
                                   char *stateFileName /* = NULL*/)
{
    SimBiController::writeToFile(fileName, stateFileName);
}

int CSimBiController::getFSMState() { return SimBiController::getFSMState(); }

Quaternion CSimBiController::getCharacterFrame()
{
    return SimBiController::getCharacterFrame();
}

/**
 * \brief           Update the d and v vectors
 * d = com_pos - stance_foot
 * v = com_vel 
 * 
 * these two vectors are in character frame
*/
void CSimBiController::updateDAndV()
{

    Vector3d com = character->getCOM(), com_vel = character->getCOMVelocity(),
             stance_foot_pos = this->stanceFoot->getCMPosition();
    // 1. calc d and v in world frame
    d = com - stance_foot_pos;
    v = com_vel;

    // 2. calcultate them in the character frame
    // change the expression from world frame to character frame
    Quaternion character_to_world = character->getHeading();
    d = character_to_world.getInverse().rotate(d);
    v = character_to_world.getInverse().rotate(v);
    std::cout << "[csbc] d = " << d << " v = " << v << " in char frame\n";
}

SimBiConState *CSimBiController::getState(uint idx)
{
    return SimBiController::getState(idx);
}

void CSimBiController::updateTrackingPose(DynamicArray<double> &trackingPose,
                                          double phiToUse /* = -1*/)
{
    SimBiController::updateTrackingPose(trackingPose, phiToUse);
}

int CSimBiController::getStance() { return SimBiController::getStance(); }

/**
 * \brief               given the phase, comepute the default d
 * \param phi           phase time
 * \param d0            default d0
*/
void CSimBiController::computeD0(double phi, Vector3d *d0)
{
    SimBiController::computeD0(phi, d0);
}

void CSimBiController::computeV0(double phi, Vector3d *v0)
{
    SimBiController::computeV0(phi, v0);
}

void CSimBiController::parseGainLine(char *line)
{
    SimBiController::parseGainLine(line);
}

void CSimBiController::setFSMStateTo(int index)
{
    if (index < 0 || (uint)index >= states.size())
    {
        FSMStateIndex = 0;
        return;
    }
    FSMStateIndex = index;
}

/**
 * \brief           transite to another state
*/
void CSimBiController::transitionToState(int stateIndex)
{
    // 1. set the FSM state by given id
    setFSMStateTo(stateIndex);

    // 2. for this new state, get the new stance by old stance
    setStance(states[FSMStateIndex]->getStateStance(this->stance));

    // reset the phase...
    this->phi = 0;
}

Vector3d CSimBiController::getForceOn(RigidBody *rb,
                                      DynamicArray<ContactPoint> *cfs)
{
    return SimBiController::getForceOn(rb, cfs);
}

Vector3d CSimBiController::getForceOnFoot(RigidBody *foot,
                                          DynamicArray<ContactPoint> *cfs)
{
    return SimBiController::getForceOnFoot(foot, cfs);
}

bool CSimBiController::isFoot(RigidBody *rb)
{
    return SimBiController::isFoot(rb);
}

bool CSimBiController::isStanceFoot(RigidBody *rb)
{
    return SimBiController::isStanceFoot(rb);
}

bool CSimBiController::isSwingFoot(RigidBody *rb)
{
    return SimBiController::isSwingFoot(rb);
}

double
CSimBiController::getStanceFootWeightRatio(DynamicArray<ContactPoint> *cfs)
{
    return SimBiController::getStanceFootWeightRatio(cfs);
}

/**
 * \brief           computethe hip's torque, then write the result to the member "torques"
*/
void CSimBiController::computeHipTorques(const Quaternion &qRootD,
                                         const Quaternion &qSwingHipD,
                                         double stanceHipToSwingHipRatio)
{
    //compute the total torques that should be applied to the root and swing hip, keeping in mind that
    //the desired orientations are expressed in the character frame
    Vector3d rootTorque;
    Vector3d swingHipTorque;

    //this is the desired orientation in world coordinates
    Quaternion qRootDW;

    if (SimGlobals::forceHeadingControl == false)
    {
        //qRootD is specified in the character frame, so just maintain the current heading
        qRootDW = characterFrame * qRootD;
    }
    else
    {
        //qRootDW needs to also take into account the desired heading
        qRootDW = Quaternion::getRotationQuaternion(SimGlobals::desiredHeading,
                                                    SimGlobals::up) *
                  qRootD;
    }

    double rootStrength = rootControlParams.strength;
    if (rootStrength < 0)
        rootStrength = 0;
    if (rootStrength > 1)
        rootStrength = 1;

    rootControlParams.strength = 1;

    //so this is the net torque that the root wants to see, in world coordinates
    rootTorque = computePDTorque(root->getOrientation(), qRootDW,
                                 root->getAngularVelocity(), Vector3d(0, 0, 0),
                                 &rootControlParams);

    RigidBody *swingHip = character->getJoint(swingHipIndex)->getChild();

    swingHipTorque = torques[swingHipIndex];

    //we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
    Vector3d rootMakeupTorque;
    for (int i = 0; i < jointCount; i++)
        if (character->getJoint(i)->getParent() == root)
            rootMakeupTorque -= torques[i];
    rootMakeupTorque -= rootTorque;

    //add to the root makeup torque the predictive torque as well (only consider the effect of the torque in the lateral plane).
    // Vector3d rootPredictiveTorque(0, 0, rootPredictiveTorqueScale * 9.8 * d.x);
    // rootMakeupTorque += characterFrame.rotate(rootPredictiveTorque);

    //assume the stance foot is in contact...
    Vector3d stanceHipTorque = torques[stanceHipIndex];

    //now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
    //to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
    std::cout << "[origin] stance hip tau = " << stanceHipTorque
              << " swing hip tau = " << swingHipTorque << std::endl;
    stanceHipTorque +=
        rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength;
    swingHipTorque +=
        rootMakeupTorque * (1 - stanceHipToSwingHipRatio) * rootStrength;

    std::cout << "[revised] stance hip tau = " << stanceHipTorque
              << " swing hip tau = " << swingHipTorque << std::endl;

    //now transform the torque to child coordinates, apply torque limits and then change it back to world coordinates
    Quaternion qStanceHip =
        character->getJoint(stanceHipIndex)->getChild()->getOrientation();
    stanceHipTorque = qStanceHip.getComplexConjugate().rotate(stanceHipTorque);
    limitTorque(&stanceHipTorque, &controlParams[stanceHipIndex]);
    stanceHipTorque = qStanceHip.rotate(stanceHipTorque);

    Quaternion qSwingHip =
        character->getJoint(swingHipIndex)->getChild()->getOrientation();
    swingHipTorque = qSwingHip.getComplexConjugate().rotate(swingHipTorque);
    limitTorque(&swingHipTorque, &controlParams[swingHipIndex]);
    swingHipTorque = qSwingHip.rotate(swingHipTorque);

    //and done...
    torques[stanceHipIndex] = stanceHipTorque;
    torques[swingHipIndex] = swingHipTorque;
    std::cout << "[final] stance hip tau = " << stanceHipTorque
              << " swing hip tau = " << swingHipTorque << std::endl;
}

void CSimBiController::resolveJoints(SimBiConState *state)
{
    SimBiController::resolveJoints(state);
}

/**
 * \brief       set stance
*/
void CSimBiController::setStance(int newStance)
{
    stance = newStance;
    if (stance == LEFT_STANCE)
    {
        stanceFoot = lFoot;
        swingFoot = rFoot;
        swingHipIndex = rHipIndex;
        stanceHipIndex = lHipIndex;
    }
    else
    {
        stanceFoot = rFoot;
        swingFoot = lFoot;
        swingHipIndex = lHipIndex;
        stanceHipIndex = rHipIndex;
    }
}

RigidBody *CSimBiController::getRBBySymbolicName(char *sName)
{
    return SimBiController::getRBBySymbolicName(sName);
}

/**
 * \brief           Calculate the current target pose
 * 
 * Strange points:
 *      1. there is a root target traj which can effect other joints
 *      2. there is a strength variable for each joint: now set all strength to 1
*/
Quaternion CSimBiController::getTargetPose(ReducedCharacterState *rc_state)
{
    for (int i = 0; i < jointCount; i++)
    {
        rc_state->setJointRelativeOrientation(Quaternion(1, 0, 0, 0), i);
        rc_state->setJointRelativeAngVelocity(Vector3d(), i);
        controlParams[i].controlled = true;
        controlParams[i].relToCharFrame = false;
    }
    Quaternion root_target(1, 0, 0, 0);
    double phi_used = std::min(1., phi);
    SimBiConState *cur_state = this->states[FSMStateIndex];
    // CON_INFO("current state {}", FSMStateIndex);
    for (int id = 0; id < cur_state->getTrajectoryCount(); id++)
    {
        // 1. in this state, get the target trajectory
        // given stance, judge the target joint
        int joint_id = cur_state->sTraj[id]->getJointIndex(stance);
        CON_INFO("[ctrl] begin to compute target pose for joint {} {}",
                 joint_id, character->getJoint(joint_id)->getName());

        // get the target d and target v
        // Vector3d d0, v0;
        // computeD0(phi_used, &d0); // get the d target
        // computeV0(phi_used, &v0); // get the v target

        // CON_INFO("[ctrl] joint {}: d0 {} v0 {}", joint_id, d0, v0);

        // get the target orientation for this joint
        Quaternion tar_orient = cur_state->sTraj[id]->evaluateTrajectory(
            this, character->getJoint(joint_id), stance, phi_used, d, v);

        if (joint_id == -1)
        {
            // for root joint, set the target
            root_target = tar_orient;
            // get the strength of root
            // rootControlParams.strength =
            //     cur_state->sTraj[id]->evaluateStrength(phi_used);
            rootControlParams.strength = 1;
            CON_INFO("root strength {}", rootControlParams.strength);
        }
        else
        {
            // if this joint is the swinghip or the given traj is relative to the character frame
            // it's equavailant to "UseWorldCoord"
            if (cur_state->sTraj[id]->relToCharFrame == true ||
                joint_id == swingHipIndex)
            {
                controlParams[joint_id].relToCharFrame = true;
                controlParams[joint_id].charFrame = characterFrame;
            }

            // set the relative orientation
            rc_state->setJointRelativeOrientation(tar_orient, joint_id);

            // evaluate strenth again
            // controlParams[joint_id].strength =
            //     cur_state->sTraj[id]->evaluateStrength(phi_used);
            controlParams[joint_id].strength = 1;
            // CON_INFO(
            // "[ctrl] joint {} strength {}",
            // character->getJoint(cur_state->sTraj[id]->getJointIndex(stance))
            //     ->getName(),
            // controlParams[joint_id].strength);
        }
    }
    return root_target;
}

/**
 * \brief           specify the fixed trajectories for each state. the old state will be deleted
*/
void CSimBiController::loadCustomFSMFromFile(const std::string &file)
{
    // 1. clear the old states
    states.clear();

    // 2. load the json
    Json::Value root;
    CON_ASSERT(JsonUtil::LoadJson(file, root));

    int num_of_states = JsonUtil::ParseAsInt("num_of_states", root);
    CON_INFO("num of FSM state {}", num_of_states);

    CustomSimBiConState *tempState = nullptr;
    const Json::Value &states_json = JsonUtil::ParseAsValue("states", root);
    CON_ASSERT(states_json.isArray());
    CON_ASSERT(states_json.size() == num_of_states);
    int state_offset = states.size();
    for (int i = 0; i < num_of_states; i++)
    {
        tempState = new CustomSimBiConState();

        tempState->readState(states_json[i], state_offset);
        resolveJoints(tempState);
        states.push_back(tempState);
    }
    // 3. load the json states, build the custom states
    transitionToState(startingState);
}