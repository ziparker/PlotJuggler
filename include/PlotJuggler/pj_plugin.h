#ifndef PJ_PLUGIN_H
#define PJ_PLUGIN_H

#include <QtPlugin>
#include <QMenu>
#include <QDomDocument>

class PlotJugglerPlugin: public QObject
{

public:
    PlotJugglerPlugin() {}

    virtual const char* name() const = 0;

    virtual bool isDebugPlugin() { return false; }

    virtual QWidget* optionsWidget() { return nullptr; }

    virtual QDomElement xmlSaveState(QDomDocument &doc) const
    {
        return QDomElement();
    }

    virtual bool xmlLoadState(const QDomElement &parent_element )
    {
        return false;
    }

    virtual void addActionsToParentMenu( QMenu* menu ) {}

private:

};

#endif // PJ_PLUGIN_H
