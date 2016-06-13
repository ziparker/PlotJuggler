#ifndef SUBWINDOW_H
#define SUBWINDOW_H

#include <QMainWindow>

class SubWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit SubWindow(QWidget *parent = 0);

signals:
    void closeRequestedByUser();

protected:
    virtual void closeEvent(QCloseEvent *event);

};

#endif // SUBWINDOW_H
