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

#include "BalanceFeedback.h"
#include "Utils/JsonUtil.h"
#include <Core/ConUtils.h>
#include <Core/SimBiController.h>
#include <Physics/Joint.h>

BalanceFeedback::BalanceFeedback(void) {}

BalanceFeedback::~BalanceFeedback(void) {}

void LinearBalanceFeedback::loadFromFile(FILE *f)
{
    if (f == NULL)
        throwError("File pointer is NULL - cannot read gain coefficients!!");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];

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
        case CON_FEEDBACK_END:
            //we're done...
            return;
            break;
        case CON_COMMENT:
            break;
        case CON_CV:
            if (sscanf(line, "%lf", &this->cv) != 1)
                throwError("A cv value must be specified!");
            break;
        case CON_CD:
            if (sscanf(line, "%lf", &this->cd) != 1)
                throwError("A cd value must be specified!");
            break;
        case CON_D_MIN:
            if (sscanf(line, "%lf", &this->dMin) != 1)
                throwError("A dMin value must be specified!");
            break;
        case CON_D_MAX:
            if (sscanf(line, "%lf", &this->dMax) != 1)
                throwError("A dMax value must be specified!");
            break;
        case CON_V_MIN:
            if (sscanf(line, "%lf", &this->vMin) != 1)
                throwError("A vMin value must be specified!");
            break;
        case CON_V_MAX:
            if (sscanf(line, "%lf", &this->vMax) != 1)
                throwError("A vMax value must be specified!");
            break;
        case CON_FEEDBACK_PROJECTION_AXIS:
            if (sscanf(line, "%lf %lf %lf", &this->feedbackProjectionAxis.x,
                       &this->feedbackProjectionAxis.y,
                       &this->feedbackProjectionAxis.z) != 3)
                throwError("The axis for a trajectory is specified by three "
                           "parameters!");
            this->feedbackProjectionAxis.toUnit();
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
    throwError("Incorrect SIMBICON input file: No \'/jointTrajectory\' found ",
               buffer);
}

/**
	This method is used to write the state parameters to a file
*/
void LinearBalanceFeedback::writeToFile(FILE *f)
{
    if (f == NULL)
        return;

    fprintf(f, "\t\t\t%s linear\n", getConLineString(CON_FEEDBACK_START));

    fprintf(f, "\t\t\t\t%s %lf %lf %lf\n",
            getConLineString(CON_FEEDBACK_PROJECTION_AXIS),
            feedbackProjectionAxis.x, feedbackProjectionAxis.y,
            feedbackProjectionAxis.z);
    fprintf(f, "\t\t\t\t%s %lf\n", getConLineString(CON_CD), cd);
    fprintf(f, "\t\t\t\t%s %lf\n", getConLineString(CON_CV), cv);
    if (dMin > -1000)
        fprintf(f, "\t\t\t\t%s %lf\n", getConLineString(CON_D_MIN), dMin);
    if (dMax < 1000)
        fprintf(f, "\t\t\t\t%s %lf\n", getConLineString(CON_D_MAX), dMax);
    if (vMin > -1000)
        fprintf(f, "\t\t\t\t%s %lf\n", getConLineString(CON_V_MIN), vMin);
    if (vMax < 1000)
        fprintf(f, "\t\t\t\t%s %lf\n", getConLineString(CON_V_MAX), vMax);

    fprintf(f, "\t\t\t%s\n", getConLineString(CON_FEEDBACK_END));
}

/**
 * \brief           load the feedback config from json
 *  1. load Cd, Cv
 *  2. load axis
*/
CustomBalanceFeedback::CustomBalanceFeedback() {}
CustomBalanceFeedback::~CustomBalanceFeedback() {}
void CustomBalanceFeedback::loadFromFile(const Json::Value &root_)
{
    mCd_lst.clear();
    mCv_lst.clear();
    mAxes_lst.clear();

    for (int i = 0; i < root_.size(); i++)
    {
        Json::Value value = root_[i];
        double cd = JsonUtil::ParseAsDouble("Cd", value);
        double cv = JsonUtil::ParseAsDouble("Cv", value);
        Vector3d balance_axis, affected_axis;
        {
            Vector axis =
                JsonUtil::ReadVectorJson(JsonUtil::ParseAsValue("axis", value));
            balance_axis.setX(axis[0]);
            balance_axis.setY(axis[1]);
            balance_axis.setZ(axis[2]);
        }
        {
            Vector axis = JsonUtil::ReadVectorJson(
                JsonUtil::ParseAsValue("affected_axis", value));
            affected_axis.setX(axis[0]);
            affected_axis.setY(axis[1]);
            affected_axis.setZ(axis[2]);
        }
        mCd_lst.push_back(cd);
        mCv_lst.push_back(cv);
        mAxes_lst.push_back(balance_axis);
        mAffectedAxes_lst.push_back(affected_axis);
    }
}

double CustomBalanceFeedback::getFeedbackContribution(SimBiController *con,
                                                      Joint *j, double phi,
                                                      Vector3d d, Vector3d v)
{

    CON_ERROR("prohibited");
    return 0;
}
void CustomBalanceFeedback::writeToFile(FILE *fp) { CON_ERROR("prohibited"); }
void CustomBalanceFeedback::loadFromFile(FILE *fp) { CON_ERROR("prohibited"); }

Quaternion CustomBalanceFeedback::getFeedbackRotation(SimBiController *con,
                                                      Joint *j, double phi,
                                                      Vector3d d, Vector3d v)
{
    Quaternion rot = Quaternion(1, 0, 0, 0);
    int num = mCd_lst.size();
    for (int i = 0; i < num; i++)
    {
        double cd = this->mCd_lst[i], cv = mCv_lst[i];
        Vector3d dot_axis = mAxes_lst[i];
        double d_offset = d.dotProductWith(dot_axis),
               v_offset = v.dotProductWith(dot_axis);
        double angle = cd * d_offset + cv * v_offset;
        Vector3d affected_axis = mAffectedAxes_lst[i];
        Quaternion rot_comp =
            Quaternion::getRotationQuaternion(angle, affected_axis);
        // std::cout << "[feedback] dot_axis = " << dot_axis
        //           << " affected_axis = " << affected_axis
        //           << " angle = " << angle << std::endl;
        rot = rot_comp * rot;
    }
    return rot;
}