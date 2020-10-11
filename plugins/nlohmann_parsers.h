#ifndef NLOHMANN_PARSERS_H
#define NLOHMANN_PARSERS_H

#include "nlohmann/json.hpp"
#include "PlotJuggler/messageparser_base.h"

class NlohmannParser: public MessageParser
{
public:

  bool parseMessageImpl(double timestamp);

protected:

  nlohmann::json _json;
};


class JSON_Parser: public NlohmannParser
{
public:

  const char* formatName() const override{
    return "JSON";
  }

  bool parseMessage(const MessageRef msg, double timestamp) override;
};

class UBJSON_Parser: public NlohmannParser
{
public:

  const char* formatName() const override{
    return "UBJSON";
  }

  bool parseMessage(const MessageRef msg, double timestamp) override;
};

class CBOR_Parser: public NlohmannParser
{
public:

  const char* formatName() const override{
    return "CBOR";
  }

  bool parseMessage(const MessageRef msg, double timestamp) override;
};

class BSON_Parser: public NlohmannParser
{
public:

  const char* formatName() const override{
    return "BSON";
  }

  bool parseMessage(const MessageRef msg, double timestamp) override;
};

class MessagePack_Parser: public NlohmannParser
{
public:

  const char* formatName() const override{
    return "MessagePack";
  }

  bool parseMessage(const MessageRef msg, double timestamp) override;
};


#endif // NLOHMANN_PARSERS_H
