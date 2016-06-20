#ifndef TABBEDPLOTWIDGET_H
#define TABBEDPLOTWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include "plotmatrix.h"

namespace Ui {
class TabbedPlotWidget;
}

class MainWindow;


class TabbedPlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TabbedPlotWidget(PlotDataMap *mapped_data, PlotMatrix* first_tab, MainWindow* main_window, QWidget *parent );

    explicit TabbedPlotWidget(PlotDataMap *mapped_data, QWidget* main_window_parent );

    void setSiblingsList( const std::map<QString,TabbedPlotWidget*>& other_tabbed_widgets );

    PlotMatrix* currentTab();

    QTabWidget* tabWidget();

    void addTab(PlotMatrix *tab);

    QDomElement xmlSaveState(QDomDocument &doc);
    bool xmlLoadState(QDomElement &tabbed_area);

    bool setStreamingMode(bool streaming_mode);

    ~TabbedPlotWidget();

private slots:

    void renameCurrentTab();

    void on_pushAddColumn_pressed();

    void on_pushVerticalResize_pressed();

    void on_pushHorizontalResize_pressed();

    void on_pushAddRow_pressed();

    void on_addTabButton_pressed();

    void on_pushremoveEmpty_pressed();

    void on_tabWidget_currentChanged(int index);

    void on_tabWidget_tabCloseRequested(int index);

    void on_buttonLinkHorizontalScale_toggled(bool checked);

    void on_requestTabMovement(const QString &destination_name);

    void moveTabIntoNewWindow();

private:
    Ui::TabbedPlotWidget *ui;

    void init();

    QAction* _action_renameTab;
    QMenu* _tab_menu;

    PlotDataMap *_mapped_data;

    QWidget* _main_window;
    bool _horizontal_link;

    QString _parent_type;

    std::map<QString,TabbedPlotWidget*> _other_siblings;

protected:
    virtual bool eventFilter(QObject *obj, QEvent *event);

signals:
    void undoableChangeHappened();
    void createTabbedDialog(bool);
    void sendTabToNewWindow(PlotMatrix *);
};

#endif // TABBEDPLOTWIDGET_H
