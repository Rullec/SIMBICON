#pragma once

#include "LogUtil.h"
#include "MathLib/Vector.h"
#include "json/json.h"
#include <string>

namespace Json
{

};
class JsonUtil
{
public:
    static Vector ReadVectorJson(const Json::Value &root);
    static bool ReadVectorJson(const Json::Value &root,
                               Vector &out_vec);
    static bool LoadJson(const std::string &path, Json::Value &value);
    static bool WriteJson(const std::string &path, Json::Value &value,
                          bool indent = true);

    static int ParseAsInt(const std::string &data_field_name,
                          const Json::Value &root);
    static std::string ParseAsString(const std::string &data_field_name,
                                     const Json::Value &root);
    static double ParseAsDouble(const std::string &data_field_name,
                                const Json::Value &root);
    static float ParseAsFloat(const std::string &data_field_name,
                              const Json::Value &root);
    static bool ParseAsBool(const std::string &data_field_name,
                            const Json::Value &root);
    static Json::Value ParseAsValue(const std::string &data_field_name,
                                    const Json::Value &root);
};
