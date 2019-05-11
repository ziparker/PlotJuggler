#ifndef MESSAGEPARSER_TEMPLATE_H
#define MESSAGEPARSER_TEMPLATE_H

#include <QtPlugin>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include "PlotJuggler/plotdata.h"

typedef std::string MessageKey;

typedef std::vector<uint8_t> RawMessage;
/**
 * @brief The MessageParser is the base class to create plugins that are able to parse one or
 * multiple Message types.
 * Each message type is uniquely identified by a MessageKey (128 bits, sufficiently large to
 * hold a MD5Sum identifier).
 *
 * You push one or more raw messages using the method pushRawMessage()
 * Once you have done, the result can be copied using plotData()
 */
class MessageParser{

public:

    virtual ~MessageParser() {}

    virtual const std::unordered_set<MessageKey>& getCompatibleMessageKeys() const = 0;

    virtual void pushRawMessage(const MessageKey& key, const RawMessage& msg, double timestamp) = 0;

    virtual void extractData(PlotDataMapRef&) = 0;

};

/// Factory to store and retrieve parsers.

class ParsersRegistry
{

public:

    void addParser(MessageParser* parser)
    {
        for( MessageKey key: parser->getCompatibleMessageKeys() )
        {
            parsers_map.insert( { key, parser } );
        }
        parsers.push_back( std::unique_ptr<MessageParser>(parser) );
    }

    const MessageParser& operator[](const MessageKey& key)
    {
        return *(parsers_map[key]);
    }
private:

    std::unordered_map<MessageKey, const MessageParser*> parsers_map;
    std::vector<std::unique_ptr<MessageParser>> parsers;
};


QT_BEGIN_NAMESPACE

#define MessageParser_iid "com.icarustechnology.PlotJuggler.MessageParser"
Q_DECLARE_INTERFACE(MessageParser, MessageParser_iid)

QT_END_NAMESPACE


#endif

