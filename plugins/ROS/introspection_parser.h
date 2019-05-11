#ifndef INTROSPECTIONPARSER_H
#define INTROSPECTIONPARSER_H

#include "PlotJuggler/messageparser_base.h"
#include <ros_type_introspection/ros_introspection.hpp>

class IntrospectionParser : public MessageParser
{
public:
    IntrospectionParser() {}

    void clear()
    {
        //TODO
    }

    void setMaxArrayPolicy(size_t max_array_size, bool discard_entire_array);

    void setUseHeaderStamp( bool use )
    {
        // TODO
    }

    void addRules( const RosIntrospection::SubstitutionRuleMap& rules)
    {
        for(const auto& it: rules)
        {
            _ros_parser.registerRenamingRules(
                        RosIntrospection::ROSType(it.first) ,
                        it.second );
        }
    }

    bool registerSchema(const std::string& topic_name,
                        RosIntrospection::ROSType type,
                        const std::string& definition)
    {
        _ros_parser.registerMessageDefinition(topic_name, type, definition);
        _registered_keys.insert( topic_name );
    }

    virtual const std::unordered_set<MessageKey>& getCompatibleMessageKeys() const override
    {
        return _registered_keys;
    }


    void pushRawMessage(const MessageKey& topic_name,
                        const RawMessage& msg,
                        double timestamp) override;

    void showWarnings();

    virtual void extractData(PlotDataMapRef& out) override
    {
        out = std::move(_plot_map);
        _plot_map = PlotDataMapRef();
    }

private:
    std::unordered_set<MessageKey> _registered_keys;
    RosIntrospection::Parser _ros_parser;
    PlotDataMapRef _plot_map;

    ParsersRegistry _parsers;

    uint32_t _max_array_size;
    bool _warnings_enabled;
    bool _discard_large_array;

    std::unordered_set<std::string> _warn_headerstamp;
    std::unordered_set<std::string> _warn_monotonic;
    std::unordered_set<std::string> _warn_cancellation;
    std::unordered_set<std::string> _warn_max_arraysize;

    double extractRealValue( const RosIntrospection::Variant& value,
                             const std::string& item_name);
};

#endif // INTROSPECTIONPARSER_H
