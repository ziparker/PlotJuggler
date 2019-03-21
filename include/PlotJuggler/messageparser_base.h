#ifndef MESSAGEPARSER_TEMPLATE_H
#define MESSAGEPARSER_TEMPLATE_H

#include <QtPlugin>
#include <array>
#include <unordered_map>
#include <functional>
#include "PlotJuggler/plotdata.h"

typedef std::array<uint8_t, 16> MessageKey;

namespace std {

  template <> struct hash<MessageKey>
  {
    std::size_t operator()(const MessageKey& key) const
    {
        const int S = sizeof(size_t);
        size_t temp = 0, h = 0;

        for (size_t i = 0; i < sizeof(key); i += S){
            memcpy(&temp, key.data() + i, S);
            h = h ^ ( std::hash<size_t>()( temp ) << 1);
        }
        return h;
    }
  };
}

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

    virtual const std::vector<MessageKey>& getCompatibleMessageKeys() const = 0;

    // return true if succesfull + an optional warning string
    virtual std::pair<bool, std::string>
    pushRawMessage(const MessageKey& key, const RawMessage& msg) = 0;

    virtual const PlotData& plotData() const
    {
        return _plot_data;
    }

    virtual PlotData& plotData()
    {
        return _plot_data;
    }

private:

    PlotData _plot_data;
};

/// Factory and Singleton to store and retrieve parsers.

class RegisteredParsers
{
public:
    typedef std::unordered_map<MessageKey, std::unique_ptr<MessageParser>> ParserMap;

    ParserMap& get()
    {
        static std::unordered_map<MessageKey, std::unique_ptr<MessageParser>> parsers;
        return parsers;
    }

    void addParser(const MessageKey& key, MessageParser* parser)
    {
        get().insert( { key, std::unique_ptr<MessageParser>(parser) });
    }

    MessageParser& operator[](const MessageKey& key)
    {
        get()[key];
    }

};


QT_BEGIN_NAMESPACE

#define MessageParser_iid "com.icarustechnology.PlotJuggler.MessageParser"
Q_DECLARE_INTERFACE(MessageParser, MessageParser_iid)

QT_END_NAMESPACE


#endif

