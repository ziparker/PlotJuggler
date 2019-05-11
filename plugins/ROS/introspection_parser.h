#ifndef INTROSPECTIONPARSER_H
#define INTROSPECTIONPARSER_H

#include "RosMsgParsers/ros_messageparser.h"
#include <ros_type_introspection/ros_introspection.hpp>

class IntrospectionParser : public MessageParser
{
public:
    IntrospectionParser();

    void clear();

    void setMaxArrayPolicy(size_t max_array_size,
                           bool discard_entire_array);

    void addRules( const RosIntrospection::SubstitutionRuleMap& rules)
    {
        for(const auto& it: rules)
        {
            _ros_parser->registerRenamingRules(
                        RosIntrospection::ROSType(it.first) ,
                        it.second );
        }
    }

    bool registerSchema(const std::string& topic_name,
                        const std::string& md5sum,
                        RosIntrospection::ROSType type,
                        const std::string& definition);

    virtual const std::unordered_set<MessageKey>& getCompatibleMessageKeys() const override
    {
        return _registered_keys;
    }


    void pushRawMessage(const MessageKey& topic_name,
                        const RawMessage& msg,
                        double timestamp) override;

    void showWarnings();

    virtual void extractData(PlotDataMapRef& destination,
                             const std::string& prefix) override;


private:
    std::unordered_set<MessageKey> _registered_keys;
    std::unique_ptr<RosIntrospection::Parser> _ros_parser;
    PlotDataMapRef _plot_map;

    std::unordered_map<std::string, MessageParser*> _builtin_parsers;

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
