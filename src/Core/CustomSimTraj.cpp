#include "CustomSimTraj.h"
#include "Utils/JsonUtil.h"
#include "Utils/LogUtil.h"
#include <iostream>

CustomTrajectoyComponent::CustomTrajectoyComponent()
{
    CON_INFO("custom traj component constructed");
}
CustomTrajectoyComponent::~CustomTrajectoyComponent() {}

/**
 * \brief               For this component
 * \brief con           controller
 * \brief j             given joint
 * \brief stance        left stance or right stance?
 * \brief phi           ref motion phase
 * \brief d             stance_foot -> com_pos
 * \brief v             com_vel
 * 
 * \return target orientation of this joint by 
*/
Quaternion CustomTrajectoyComponent::evaluateTrajectoryComponent(
    SimBiController *con, Joint *j, int stance, double phi, const Vector3d &d,
    const Vector3d &v)
{
    double baseAngle = offset;
    // CON_INFO("[traj_comp] joint {} base_angle offset = {}", j->getName(),
    //          baseAngle);
    if (baseTraj.getKnotCount() > 0)
    {
        // =========== old method ==========
        // change this catmull rom to the lager index value
        // if it exceeds the max value, we simply use the max value
        // baseAngle += baseTraj.evaluate_catmull_rom(phi);

        // =========== new method ===========
        const int num_of_intervals = 3;
        baseAngle += getNextKnotValue(phi, num_of_intervals);

        // CON_INFO("[traj_comp] joint {} catmull offset = {}", j->getName(),
        //          baseTraj.evaluate_catmull_rom(phi));
    }

    if (stance == LEFT_STANCE && reverseAngleOnLeftStance)
    {
        baseAngle = -baseAngle;
        std::cout << "left stance, reverse angle on left stance, base angle "
                     "opposite: "
                  << baseAngle << std::endl;
    }

    if (stance == RIGHT_STANCE && reverseAngleOnRightStance)
    {
        baseAngle = -baseAngle;
        std::cout << "right stance, reverse angle on right stance, base angle "
                     "opposite: "
                  << baseAngle << std::endl;
    }
    double final_angle = baseAngle;
    Quaternion final_rot = Quaternion(1, 0, 0, 0);
    std::cout << "[eval] joint " << j->getName()
              << " base_angle = " << final_angle << std::endl;
#ifdef USE_CUSTOM_FSM
    Quaternion feedback_rot = computeCustomFeedback(con, j, phi, d, v);
    final_rot = feedback_rot *
                Quaternion::getRotationQuaternion(final_angle, rotationAxis);
    std::cout << "[eval] joint " << j->getName()
              << " final_angle = " << final_rot << std::endl;
#endif
#ifndef USE_CUSTOM_FSM
    final_angle += computeFeedback(con, j, phi, d, v);
    final_rot = Quaternion::getRotationQuaternion(final_angle, rotationAxis);
#endif
    // CON_INFO("[traj_comp] joint {} comp base_angle {}, feedback_value {}, "
    //          "final_angle {}",
    //          j->getName(), baseAngle, feedbackValue, final_angle);
    return final_rot;
}

/**
 * \brief               given motion phase, given d and v, calculate feedback value ()
 * \param con           simbicon controller
 * \param j             joint
 * \param phi           motion phase
 * \param d             given stance foot to com pos, vector
 * \param v             given com vel, vector
*/
double CustomTrajectoyComponent::computeFeedback(SimBiController *con, Joint *j,
                                                 double phi, const Vector3d &d,
                                                 const Vector3d &v)
{
    if (bFeedback == NULL)
        return 0;
    return bFeedback->getFeedbackContribution(con, j, phi, d, v);
}

void CustomTrajectoyComponent::updateComponent(
    SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
    Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX, Trajectory1D &newVTrajZ,
    Trajectory1D *oldDTrajX, Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
    Trajectory1D *oldVTrajZ, int nbSamples)
{
    CON_ERROR("update component, need to check again");
    TrajectoryComponent::updateComponent(
        con, j, newDTrajX, newDTrajZ, newVTrajX, newVTrajZ, oldDTrajX,
        oldDTrajZ, oldVTrajX, oldVTrajZ, nbSamples);
}

// void CustomTrajectoyComponent::readBaseTrajectory(FILE *f) {}

void CustomTrajectoyComponent::readTrajectoryComponent(FILE *f)
{
    TrajectoryComponent::readTrajectoryComponent(f);
}
#include "Trajectory.h"
/**
 * \brief           read the trajectory component from the json value
*/
void CustomTrajectoyComponent::readTrajectoryComponent(const Json::Value &value)
{
    // 1. load the target pose
    Vector rot_axis_json =
        JsonUtil::ReadVectorJson(JsonUtil::ParseAsValue("axis", value));
    rotationAxis.setX(rot_axis_json[0]);
    rotationAxis.setY(rot_axis_json[1]);
    rotationAxis.setZ(rot_axis_json[2]);
    double angle = JsonUtil::ParseAsDouble("angle", value);
    baseTraj.addKnot(0, angle);
    baseTraj.addKnot(1, angle);

    std::cout << "[debug] comp axis = " << rotationAxis
              << " tar angle = " << angle << std::endl;
    // 2. check whether there are any feedback
    for (int i = 0; i < value.getMemberNames().size(); i++)
    {
        std::string item_name = value.getMemberNames()[i];
        if (item_name == "feedback")
        {
            // there exist the feedback, load the linear feedback
            CustomBalanceFeedback *feedback = new CustomBalanceFeedback();

            feedback->loadFromFile(value[item_name]);
            bFeedback = feedback;
            break;
        }
    }
}

void CustomTrajectoyComponent::writeBaseTrajectory(FILE *f)
{
    TrajectoryComponent::writeBaseTrajectory(f);
}

void CustomTrajectoyComponent::writeTrajectoryComponent(FILE *f)
{
    TrajectoryComponent::writeTrajectoryComponent(f);
}

/**
 * \brief               given phi, calculate the interval
*/
double CustomTrajectoyComponent::getNextKnotValue(const double phi,
                                                  int num_of_interval)
{
    double min_phi = 0, max_phi = 1;
    CON_ASSERT(phi <= max_phi && phi >= min_phi);
    CON_ASSERT(num_of_interval >= 2);

    double interval = (max_phi - min_phi) / num_of_interval;
    double cur_pos = -1;
    for (int i = 0; i < num_of_interval; i++)
    {
        cur_pos = i * interval;
        if (cur_pos > phi)
            break;
    }

    CON_ASSERT(cur_pos > 0);
    // std::cout << "[base_traj] cur pos = " << cur_pos << std::endl;
    return baseTraj.evaluate_catmull_rom(cur_pos);
}
// ====================================================

CustomTrajectory::CustomTrajectory() {}
CustomTrajectory::~CustomTrajectory() {}

/**
 * \brief               given phase and stance, calculate
 * \param con           simbicon controller
 * \param j             target joint
 * \param stance        left stance or right stance?
 * \param phi           ref motion phase
 * \param d             precalculated d (stance_foot -> com_pos)
 * \param v             precalculated v (com_vel)
*/
Quaternion CustomTrajectory::evaluateTrajectory(SimBiController *con, Joint *j,
                                                int stance, double phi,
                                                const Vector3d &d,
                                                const Vector3d &v)
{
    Quaternion q = Quaternion(1, 0, 0, 0);

    // for each component, calculate trajectory
    for (uint i = 0; i < components.size(); i++)
    {
        auto &cur_comp = components[i];
        Quaternion cur_comp_orient =
            cur_comp->evaluateTrajectoryComponent(con, j, stance, phi, d, v);
        // CON_INFO("[traj] joint {} component {} orientation {}", j->getName(), i,
        //          cur_comp_orient);
        q = cur_comp_orient * q;
    }
    // CON_INFO("[traj] joint {} final orientation {}", j->getName(), q);
    return q;
}

double CustomTrajectory::evaluateStrength(double phiToUse)
{
    CON_ERROR("evaluate strength");
    return Trajectory::evaluateStrength(phiToUse);
}
int CustomTrajectory::getJointIndex(int stance)
{
    return Trajectory::getJointIndex(stance);
}
void CustomTrajectory::updateComponents(
    SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
    Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX, Trajectory1D &newVTrajZ,
    Trajectory1D *oldDTrajX, Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
    Trajectory1D *oldVTrajZ, int nbSamples)

{
}
void CustomTrajectory::readTrajectory(FILE *f)
{
    Trajectory::readTrajectory(f);
}
void CustomTrajectory::writeStrengthTrajectory(FILE *f)
{
    Trajectory::writeStrengthTrajectory(f);
}
void CustomTrajectory::writeTrajectory(FILE *f)
{
    Trajectory::writeTrajectory(f);
}

/**
 * \brief           given the target angle of a specified joint, (whose name has been set to TrajName)
 *              create the target trajectory, and trajectory components
*/
void CustomTrajectory::readTrajectory(const Json::Value &value)
{
    CON_INFO("read traj for joint {}", this->jName);
    // std::cout << value << std::endl;
    components.clear();
    CustomTrajectoyComponent *comp = new CustomTrajectoyComponent();
    comp->readTrajectoryComponent(value);
    components.push_back(comp);
}

Quaternion CustomTrajectoyComponent::computeCustomFeedback(SimBiController *con,
                                                           Joint *j, double phi,
                                                           const Vector3d &d,
                                                           const Vector3d &v)
{
    if (bFeedback == nullptr)
        return Quaternion(1, 0, 0, 0);
    else
    {
        auto cust_feedback =
            dynamic_cast<CustomBalanceFeedback *>(this->bFeedback);

        CON_ASSERT(cust_feedback != nullptr);
        return cust_feedback->getFeedbackRotation(con, j, phi, d, v);
    }
}