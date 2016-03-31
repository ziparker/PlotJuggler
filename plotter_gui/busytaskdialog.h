#ifndef BUSYTASKDIALOG_H
#define BUSYTASKDIALOG_H

#include <QObject>
#include <QProgressDialog>

class BusyTaskDialog : public QProgressDialog
{
    Q_OBJECT
public:
    explicit BusyTaskDialog(QString text);

signals:

public slots:

private:

public slots:
};



#endif // BUSYTASKDIALOG_H
