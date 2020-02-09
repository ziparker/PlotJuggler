#include "lua_custom_function.h"
#include "qml_custom_function.h"
#include <QSettings>


std::unique_ptr<CustomFunction>
CustomFunctionFactory(const std::string &linkedPlot,
                      const std::string &plotName,
                      const QString &globalVars,
                      const QString &function)
{
  SnippetData snippet = { QString::fromStdString(plotName), globalVars, function };
  return CustomFunctionFactory(linkedPlot, snippet);
}

std::unique_ptr<CustomFunction>
CustomFunctionFactory(const std::string &linkedPlot,
                      const SnippetData &snippet)
{
  QSettings settings;
  static bool is_qml = settings.value("CustomFunction/language", "qml").toString() == "qml";
  if( is_qml)
  {
    return std::make_unique<QmlCustomFunction>( linkedPlot, snippet );
  }
  else{
    return std::make_unique<LuaCustomFunction>( linkedPlot, snippet );
  }
}
