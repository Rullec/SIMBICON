#include "JsonUtil.h"
#include "LogUtil.h"
#include <fstream>
#include <iostream>
#include <memory>
Vector JsonUtil::ReadVectorJson(const Json::Value &root)
{
    Vector xd;
    JsonUtil::ReadVectorJson(root, xd);
    return xd;
}
bool JsonUtil::ReadVectorJson(const Json::Value &root, Vector &out_vec)
{
    bool succ = false;
    int num_vals = root.size();

    if (root.isArray())
    {
        out_vec.resize(num_vals);
        for (int i = 0; i < num_vals; ++i)
        {
            Json::Value json_elem = root.get(i, 0);
            out_vec[i] = json_elem.asDouble();
        }
        succ = true;
    }

    return succ;
}

bool JsonUtil::LoadJson(const std::string &path, Json::Value &value)
{
    // cFileUtil::AddLock(path);
    // std::cout <<"parsing " << path << " begin \n";
    std::ifstream fin(path);
    if (fin.fail() == true)
    {
        std::cout << "[error] JsonUtil::LoadJson file " << path
                  << " doesn't exist\n";
        return false;
    }
    Json::CharReaderBuilder rbuilder;
    std::string errs;
    bool parsingSuccessful =
        Json::parseFromStream(rbuilder, fin, &value, &errs);
    if (!parsingSuccessful)
    {
        // report to the user the failure and their locations in the
        // document.
        std::cout << "[error] JsonUtil::LoadJson: Failed to parse json\n"
                  << errs << std::endl;
        return false;
    }
    // std::cout <<"parsing " << path << " end \n";
    // cFileUtil::DeleteLock(path);
    return true;
}

bool JsonUtil::WriteJson(const std::string &path, Json::Value &value,
                         bool indent /* = true*/)
{
    // cFileUtil::AddLock(path);
    Json::StreamWriterBuilder builder;
    if (indent == false)
        builder.settings_["indentation"] = "";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream fout(path);
    if (fout.fail() == true)
    {
        CON_ERROR("WriteJson open {} failed", path);
        exit(1);
    }
    writer->write(value, &fout);
    fout.close();
    // cFileUtil::DeleteLock(path);
    return fout.fail() == false;
}

#define JSONUTIL_ASSERT_NULL(root, data) (root.isMember(data))

int JsonUtil::ParseAsInt(const std::string &data_field_name,
                         const Json::Value &root)
{
    if (false == JSONUTIL_ASSERT_NULL(root, data_field_name))
    {
        CON_ERROR("ParseAsInt {} failed", data_field_name.c_str());
        exit(0);
    }
    return root[data_field_name].asInt();
}

std::string JsonUtil::ParseAsString(const std::string &data_field_name,
                                    const Json::Value &root)
{
    if (false == JSONUTIL_ASSERT_NULL(root, data_field_name))
    {
        CON_ERROR("ParseAsString {} failed", data_field_name.c_str());
        exit(0);
    }
    return root[data_field_name].asString();
}

double JsonUtil::ParseAsDouble(const std::string &data_field_name,
                               const Json::Value &root)
{
    if (false == JSONUTIL_ASSERT_NULL(root, data_field_name))
    {
        CON_ERROR("ParseAsDouble {} failed", data_field_name.c_str());
        exit(0);
    }
    return root[data_field_name].asDouble();
}

float JsonUtil::ParseAsFloat(const std::string &data_field_name,
                             const Json::Value &root)
{
    if (false == JSONUTIL_ASSERT_NULL(root, data_field_name))
    {
        CON_ERROR("ParseAsFloat {} failed", data_field_name.c_str());
        exit(0);
    }
    return root[data_field_name].asFloat();
}

bool JsonUtil::ParseAsBool(const std::string &data_field_name,
                           const Json::Value &root)
{
    if (false == JSONUTIL_ASSERT_NULL(root, data_field_name))
    {
        CON_ERROR("ParseAsBool {} failed", data_field_name.c_str());
        exit(0);
    }
    return root[data_field_name].asBool();
}

Json::Value JsonUtil::ParseAsValue(const std::string &data_field_name,
                                   const Json::Value &root)
{
    if (false == JSONUTIL_ASSERT_NULL(root, data_field_name))
    {
        CON_ERROR("ParseAsValue {} failed", data_field_name.c_str());
        exit(0);
    }
    return root[data_field_name];
}