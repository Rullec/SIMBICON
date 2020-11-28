#include "CustomSimBiConState.h"
#include "CustomSimTraj.h"
#include "Utils/JsonUtil.h"
#include "Utils/LogUtil.h"
CustomSimBiConState::CustomSimBiConState() : SimBiConState() { myStateId = 0; }

/**
 * \brief           given the current stance (left or right), return the new stance
*/
int CustomSimBiConState::getStateStance(int oldStance)
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
 * \brief           get the expected time consumption
*/
double CustomSimBiConState::getStateTime()
{
    return SimBiConState::getStateTime();
}

/**
 * \brief           get myself's state index
*/
int CustomSimBiConState::getStateIndex() { return myStateId; }

/**
 * \brief           this method is used to retrieve the index of the next state 
*/
int CustomSimBiConState::getNextStateIndex()
{
    return SimBiConState::getNextStateIndex();
}

int CustomSimBiConState::getTrajectoryCount()
{
    return SimBiConState::getTrajectoryCount();
}

Trajectory *CustomSimBiConState::getTrajectory(uint idx)
{
    return SimBiConState::getTrajectory(idx);
}

Trajectory *CustomSimBiConState::getTrajectory(const char *name)
{
    return SimBiConState::getTrajectory(name);
}

/**
 * \brief               judge whether do we need to do transition
*/
bool CustomSimBiConState::needTransition(double phi,
                                         double swingFootVerticalForce,
                                         double stanceFootVerticalForce)
{
    // CON_ASSERT(transitionOnFootContact == true);
    // CON_INFO("we ignore the phi and min time threshold in needTransition, phi "
    //          "{} swing {} stance {}",
    //          phi, swingFootVerticalForce, stanceFootVerticalForce);
    // if ((phi > minPhiBeforeTransitionOnFootContact &&
    //      swingFootVerticalForce > minSwingFootForceForContact) ||
    //     phi >= 1)
    if (transitionOnFootContact)
    {
        if ((phi > minPhiBeforeTransitionOnFootContact &&
             swingFootVerticalForce > minSwingFootForceForContact) ||
            phi >= 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // judge by time
        if (phi >= 1)
            return true;
        else
            return false;
    }
}

const char *CustomSimBiConState::getDescription()
{
    return SimBiConState::getDescription();
}

void CustomSimBiConState::readState(FILE *f, int offset)
{
    SimBiConState::readState(f, offset);
}

/**
 * \brief           read custom simbicon state from json
 * what do we need to read?
 *  1. next state id
 *  2. description
 *  3. elasped time
 *  4. keep, reverse, left or right stance?
 *  5. transition way. foot down or timeup? next state id?
 *  6. load target custom trajectory to sTraj
 *  7. D traj and V traj (optional), now we don't do it
 * 
*/
void CustomSimBiConState::readState(const Json::Value &value, int offset)
{
    CON_ASSERT(value.empty() == false);

    // 1. state id
    myStateId = JsonUtil::ParseAsInt("state_id", value);

    // 2. description
    memset(
        description, '\0',
        sizeof(
            description)); // here the description is a fixed length array, sizeof return the total bytes number it occuys

    // 3. state time
    stateTime = JsonUtil::ParseAsDouble("elapsed_time", value);

    // 4. stance update mode
    reverseStance = false;
    keepStance = false;
    std::string stance_mode =
        JsonUtil::ParseAsString("stance_update_mode", value);
    if (stance_mode == "same")
        keepStance = true;
    else if (stance_mode == "reverse")
        reverseStance = true;
    else if (stance_mode == "left")
        stateStance = LEFT_STANCE;
    else if (stance_mode == "right")
        stateStance = RIGHT_STANCE;
    else
        CON_ERROR("invalid stance mode = {}", stance_mode);

    // 5. parse transition way
    {
        Json::Value conds = JsonUtil::ParseAsValue("conditions", value);
        CON_ASSERT(conds.size() == 1);
        Json::Value cond = conds[0];
        std::string transition_type =
            JsonUtil::ParseAsString("condition_type", cond);
        transitionOnFootContact = false;
        if (transition_type == "LinkContact")
        {
            transitionOnFootContact = true;
        }
        else if (transition_type == "ElapsedTime")
        {
            // default
        }
        else
        {
            CON_ERROR("invalid transition type {}", transition_type);
        }
        nextStateIndex = JsonUtil::ParseAsInt("target_state_id", cond);
    }

    // 6. target pose for swing/stance hip, knee, ankle
    {
        Json::Value trajs = JsonUtil::ParseAsValue("trajectories", value);
        buildTrajectories(trajs);
    }
}

void CustomSimBiConState::writeState(FILE *f, int index)
{
    SimBiConState::writeState(f, index);
}

void CustomSimBiConState::updateDVTrajectories(SimBiController *con, Joint *j,
                                               Trajectory1D &newDTrajX,
                                               Trajectory1D &newDTrajZ,
                                               Trajectory1D &newVTrajX,
                                               Trajectory1D &newVTrajZ,
                                               int nbSamples = 100)
{
    CON_ERROR("update dv trajectories");
}

/**
 * \brief               Given json target pose, build the base trajectory, or the trajectory component for this state
 * 
 * in json FSM model, there is only a single target pose for each joint in this state, so the trajectory component will keeps the same in a single result
*/
void CustomSimBiConState::buildTrajectories(const Json::Value &value)
{
    CON_ASSERT(value.empty() == false);
    // for the trajectory for each joint
    // then put the result into the array
    // std::cout << "value = " << value << std::endl;
    int size = value.size();
    sTraj.clear();
    CustomTrajectory *traj = nullptr;
    std::vector<std::string> mem_names = value.getMemberNames();

    CON_INFO("state include {} trajectories", size);

    for (int i = 0; i < size; i++)
    {
        traj = new CustomTrajectory();
        std::string joint_name = mem_names[i];
        std::cout << "for traj " << i << " joint name = " << joint_name
                  << std::endl;
        memset(traj->jName, 0, sizeof(traj->jName));
        memcpy(traj->jName, joint_name.c_str(), joint_name.size());
        traj->readTrajectory(JsonUtil::ParseAsValue(joint_name, value));
        sTraj.push_back(traj);
    }
}