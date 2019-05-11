#ifndef MESSAGEPARSER_TEMPLATE_H
#define MESSAGEPARSER_TEMPLATE_H

#include <QtPlugin>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <absl/types/span.h>
#include "PlotJuggler/plotdata.h"

typedef std::string MessageKey;

typedef absl::Span<uint8_t> RawMessage;
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

    virtual void pushRawMessage(const MessageKey& key,
                                const RawMessage& msg,
                                double timestamp) = 0;

    virtual void extractData(PlotDataMapRef& destination,
                             const std::string& prefix) = 0;

protected:

    static void appendData(PlotDataMapRef& destination_plot_map,
                           const std::string& field_name,
                           PlotData& in_data)
    {
        if( in_data.size() == 0 )
        {
            return;
        }
        auto plot_pair = destination_plot_map.numeric.find( field_name );
        if( (plot_pair == destination_plot_map.numeric.end()) )
        {
            plot_pair = destination_plot_map.addNumeric( field_name );
            plot_pair->second.swapData( in_data );
        }
        else{
            PlotData& plot_data = plot_pair->second;
            for(size_t i=0; i < in_data.size(); i++)
            {
                plot_data.pushBack( in_data[i] );
            }
        }
        in_data.clear();
    }
};

QT_BEGIN_NAMESPACE

#define MessageParser_iid "com.icarustechnology.PlotJuggler.MessageParser"
Q_DECLARE_INTERFACE(MessageParser, MessageParser_iid)

QT_END_NAMESPACE


#endif

