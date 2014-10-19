#pragma once
#include "json.h"
class JsonSerializable
{
public:

    virtual ~JsonSerializable(void)
    {
    };

    virtual void serialize( Json::Value& root ) =0;
    virtual void deserialize( Json::Value& root) =0;

};

