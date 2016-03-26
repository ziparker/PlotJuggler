#ifndef SELECTXAXISDIALOG_H
#define SELECTXAXISDIALOG_H

#include <QDialog>

namespace Ui {
class selectXAxisDialog;
}

class selectXAxisDialog : public QDialog
{
    Q_OBJECT

public:
    explicit selectXAxisDialog(QVector<QStringRef> fields, QWidget *parent = 0);
    ~selectXAxisDialog();

private slots:
    void on_buttonBox_accepted();

    void on_listFieldsWidget_currentRowChanged(int currentRow);

    void on_listFieldsWidget_doubleClicked(const QModelIndex &index);

private:
    Ui::selectXAxisDialog *ui;

};

#endif // SELECTXAXISDIALOG_H
