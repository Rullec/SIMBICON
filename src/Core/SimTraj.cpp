#include "SimTraj.h"
#include "CustomSimTraj.h"
#include "SimBiConState.h"
#include "SimBiController.h"
TrajectoryComponent::TrajectoryComponent()
{
    rotationAxis = Vector3d();
    reverseAngleOnLeftStance = false;
    reverseAngleOnRightStance = false;
    bFeedback = NULL;
    offset = 0;
}

TrajectoryComponent::~TrajectoryComponent() { delete bFeedback; }

Quaternion TrajectoryComponent::evaluateTrajectoryComponent(
    SimBiController *con, Joint *j, int stance, double phi, const Vector3d &d,
    const Vector3d &v)
{
    double baseAngle = offset;
    if (baseTraj.getKnotCount() > 0)
        baseAngle += baseTraj.evaluate_catmull_rom(phi);

    if (stance == LEFT_STANCE && reverseAngleOnLeftStance)
        baseAngle = -baseAngle;
    if (stance == RIGHT_STANCE && reverseAngleOnRightStance)
        baseAngle = -baseAngle;

    double feedbackValue = computeFeedback(con, j, phi, d, v);

    return Quaternion::getRotationQuaternion(baseAngle + feedbackValue,
                                             rotationAxis);
}

double TrajectoryComponent::computeFeedback(SimBiController *con, Joint *j,
                                            double phi, const Vector3d &d,
                                            const Vector3d &v)
{
    if (bFeedback == NULL)
        return 0;
    return bFeedback->getFeedbackContribution(con, j, phi, d, v);
}

Trajectory::Trajectory()
{
    leftStanceIndex = rightStanceIndex = -1;
    strcpy(jName, "NoNameJoint");
    strengthTraj = NULL;
    relToCharFrame = false;
}

Trajectory::~Trajectory()
{
    for (uint i = 0; i < components.size(); i++)
        delete components[i];
}

Quaternion Trajectory::evaluateTrajectory(SimBiController *con, Joint *j,
                                          int stance, double phi,
                                          const Vector3d &d, const Vector3d &v)
{
    Quaternion q = Quaternion(1, 0, 0, 0);

    for (uint i = 0; i < components.size(); i++)
        q = components[i]->evaluateTrajectoryComponent(con, j, stance, phi, d,
                                                       v) *
            q;

    return q;
}

double Trajectory::evaluateStrength(double phiToUse)
{
    if (strengthTraj == NULL)
        return 1.0;
    return strengthTraj->evaluate_catmull_rom(phiToUse);
}

int Trajectory::getJointIndex(int stance)
{
    return (stance == LEFT_STANCE) ? (leftStanceIndex) : (rightStanceIndex);
}

void Trajectory::updateComponents(
    SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
    Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX, Trajectory1D &newVTrajZ,
    Trajectory1D *oldDTrajX, Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
    Trajectory1D *oldVTrajZ, int nbSamples)
{
    int nbComponents = components.size();
    for (int i = 0; i < nbComponents; i++)
        components[i]->updateComponent(con, j, newDTrajX, newDTrajZ, newVTrajX,
                                       newVTrajZ, oldDTrajX, oldDTrajZ,
                                       oldVTrajX, oldVTrajZ, nbSamples);
}

/**
	This method is used to write the knots of a strength trajectory to the file, where they are specified one (knot) on a line
*/
void Trajectory::writeStrengthTrajectory(FILE *f)
{
    if (f == NULL || strengthTraj == NULL)
        return;

    fprintf(f, "\t\t\t%s\n", getConLineString(CON_STRENGTH_TRAJECTORY_START));

    for (int i = 0; i < strengthTraj->getKnotCount(); ++i)
    {
        fprintf(f, "\t\t\t\t%lf %lf\n", strengthTraj->getKnotPosition(i),
                strengthTraj->getKnotValue(i));
    }

    fprintf(f, "\t\t\t%s\n", getConLineString(CON_STRENGTH_TRAJECTORY_END));
}

/**
	This method is used to write a trajectory to a file
*/
void Trajectory::writeTrajectory(FILE *f)
{
    if (f == NULL)
        return;

    fprintf(f, "\t%s %s\n", getConLineString(CON_TRAJECTORY_START), jName);

    if (relToCharFrame)
        fprintf(f, "\t%s\n", getConLineString(CON_CHAR_FRAME_RELATIVE));

    if (strengthTraj != NULL)
        writeStrengthTrajectory(f);

    for (uint i = 0; i < components.size(); ++i)
    {
        fprintf(f, "\n");
        components[i]->writeTrajectoryComponent(f);
    }

    fprintf(f, "\t%s\n", getConLineString(CON_TRAJECTORY_END));
}

/**
	This method is used to read a trajectory from a file
*/
void Trajectory::readTrajectory(FILE *f)
{
    if (f == NULL)
        throwError("File pointer is NULL - cannot read gain coefficients!!");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];

    TrajectoryComponent *newComponent;

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
        case CON_STRENGTH_TRAJECTORY_START:
            //read in the base trajectory
            if (strengthTraj != NULL)
                throwError("Two strength trajectory, this is illegal!");
            strengthTraj = new Trajectory1D();
            SimBiConState::readTrajectory1D(f, *strengthTraj,
                                            CON_STRENGTH_TRAJECTORY_END);
            break;
        case CON_TRAJECTORY_END:
            //we're done...
            return;
            break;
        case CON_CUSTOM_TRAJECTORY_END:
            return;
            break;
        case CON_CHAR_FRAME_RELATIVE:
            relToCharFrame = true;
            break;
        case CON_COMMENT:
            break;
        case CON_TRAJ_COMPONENT:
            //read in the base trajectory
            newComponent = new TrajectoryComponent();
            newComponent->readTrajectoryComponent(f);
            components.push_back(newComponent);
            break;
        case CON_CUSTOM_TRAJECTORY_START:
            newComponent = new CustomTrajectoyComponent();
            newComponent->readTrajectoryComponent(f);
            components.push_back(newComponent);
        case CON_NOT_IMPORTANT:
            tprintf("Ignoring input line: \'%s\'\n", line);
            break;
        case CON_CUSTOM_TRAJ_COMPONENT_START:
            newComponent = new CustomTrajectoyComponent();
            newComponent->readTrajectoryComponent(f);
            components.push_back(newComponent);
            break;
        default:
            throwError(
                "Incorrect SIMBICON input file: \'%s\' - unexpected line.",
                buffer);
        }
    }
    throwError("Incorrect SIMBICON input file: No \'/trajectory\' found ",
               buffer);
}

/** 
	Update this component to recenter it around the new given D and V trajectories
*/
void TrajectoryComponent::updateComponent(
    SimBiController *con, Joint *j, Trajectory1D &newDTrajX,
    Trajectory1D &newDTrajZ, Trajectory1D &newVTrajX, Trajectory1D &newVTrajZ,
    Trajectory1D *oldDTrajX, Trajectory1D *oldDTrajZ, Trajectory1D *oldVTrajX,
    Trajectory1D *oldVTrajZ, int nbSamples)
{

    if (bFeedback == NULL)
        return;

    double startPhi = 0;
    double endPhi = 0;
    if (baseTraj.getKnotCount() > 0)
    {
        startPhi = std::min(startPhi, baseTraj.getMinPosition());
        endPhi = std::min(startPhi, baseTraj.getMaxPosition());
    }
    if (newDTrajX.getKnotCount() > 0)
    {
        startPhi = std::max(startPhi, newDTrajX.getMinPosition());
        endPhi = std::max(startPhi, newDTrajX.getMaxPosition());
    }

    Trajectory1D result;
    Vector3d d0, v0, newD0, newV0;
    for (int i = 0; i < nbSamples; ++i)
    {
        double interp = (double)i / (nbSamples - 1.0);
        double phi = startPhi * (1.0 - interp) + endPhi * interp;

        double baseAngle = 0;
        if (baseTraj.getKnotCount() > 0)
            baseAngle = baseTraj.evaluate_catmull_rom(phi);
        SimBiController::computeDorV(phi, &newDTrajX, &newDTrajZ, LEFT_STANCE,
                                     &newD0);
        SimBiController::computeDorV(phi, &newVTrajX, &newVTrajZ, LEFT_STANCE,
                                     &newV0);
        SimBiController::computeDorV(phi, oldDTrajX, oldDTrajZ, LEFT_STANCE,
                                     &d0);
        SimBiController::computeDorV(phi, oldVTrajX, oldVTrajZ, LEFT_STANCE,
                                     &v0);

        double feedback = computeFeedback(con, j, phi, newD0 - d0, newV0 - v0);

        if (reverseAngleOnLeftStance)
            baseAngle -= feedback;
        else
            baseAngle += feedback;

        result.addKnot(phi, baseAngle);
    }
    result.simplify_catmull_rom(0.005);
    baseTraj.copy(result);
}

/**
	This method is used to read the knots of a base trajectory from the file, where they are specified one (knot) on a line
*/
void TrajectoryComponent::writeBaseTrajectory(FILE *f)
{
    if (f == NULL)
        return;

    fprintf(f, "\t\t\t%s\n", getConLineString(CON_BASE_TRAJECTORY_START));

    for (int i = 0; i < baseTraj.getKnotCount(); ++i)
    {
        fprintf(f, "\t\t\t\t%lf %lf\n", baseTraj.getKnotPosition(i),
                baseTraj.getKnotValue(i));
    }

    fprintf(f, "\t\t\t%s\n", getConLineString(CON_BASE_TRAJECTORY_END));
}

/**
	This method is used to write a trajectory to a file
*/
void TrajectoryComponent::writeTrajectoryComponent(FILE *f)
{
    if (f == NULL)
        return;

    fprintf(f, "\t\t%s\n", getConLineString(CON_TRAJ_COMPONENT));

    fprintf(f, "\t\t\t%s %lf %lf %lf\n", getConLineString(CON_ROTATION_AXIS),
            rotationAxis.x, rotationAxis.y, rotationAxis.z);

    if (reverseAngleOnLeftStance)
        fprintf(f, "\t\t\t%s left\n",
                getConLineString(CON_REVERSE_ANGLE_ON_STANCE));
    else if (reverseAngleOnRightStance)
        fprintf(f, "\t\t\t%s right\n",
                getConLineString(CON_REVERSE_ANGLE_ON_STANCE));

    if (bFeedback)
        bFeedback->writeToFile(f);

    writeBaseTrajectory(f);

    fprintf(f, "\t\t%s\n", getConLineString(CON_TRAJ_COMPONENT_END));
}

/**
	This method is used to read a trajectory from a file
*/
void TrajectoryComponent::readTrajectoryComponent(FILE *f)
{
    if (f == NULL)
        throwError("File pointer is NULL - cannot read gain coefficients!!");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    char tmpString[200];

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
        case CON_TRAJ_COMPONENT_END:
            //we're done...
            return;
            break;
        case CON_CUSTOM_TRAJ_COMPONENT_END:
            return;
            break;
        case CON_COMMENT:
            break;
        case CON_ROTATION_AXIS:
            if (sscanf(line, "%lf %lf %lf", &this->rotationAxis.x,
                       &this->rotationAxis.y, &this->rotationAxis.z) != 3)
                throwError("The axis for a trajectory is specified by three "
                           "parameters!");
            this->rotationAxis.toUnit();
            break;
        case CON_FEEDBACK_START:
            //read the kind of feedback that is applicable to this state
            if (sscanf(line, "%s", tmpString) != 1)
                throwError("The kind of feedback to be used for a trajectory "
                           "must be specified (e.g. linear)");
            delete bFeedback;
            bFeedback = NULL;
            if (strncmp(tmpString, "linear", 6) == 0)
            {
                bFeedback = new LinearBalanceFeedback();
                bFeedback->loadFromFile(f);
            }
            else
                throwError("Unrecognized type of feedback: \'%s\'", line);
            break;
        case CON_BASE_TRAJECTORY_START:
            //read in the base trajectory
            SimBiConState::readTrajectory1D(f, baseTraj,
                                            CON_BASE_TRAJECTORY_END);
            break;
        case CON_REVERSE_ANGLE_ON_STANCE:
            if (strncmp(trim(line), "left", 4) == 0)
                reverseAngleOnLeftStance = true;
            else if (strncmp(trim(line), "right", 5) == 0)
                reverseAngleOnRightStance = true;
            else
                throwError("When using the \'startingStance\' keyword, "
                           "\'left\' or \'right\' must be specified!");
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
    throwError("Incorrect SIMBICON input file: No \'/trajectory\' found ",
               buffer);
}
