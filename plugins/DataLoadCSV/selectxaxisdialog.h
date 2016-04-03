#ifndef SELECTXAXISDIALOG_H
#define SELECTXAXISDIALOG_H

#include <QDialog>

namespace Ui {
class SelectXAxisDialog;
}

class SelectXAxisDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SelectXAxisDialog(QStringList* fields, QWidget *parent = 0);
    ~SelectXAxisDialog();

    int getSelectedRowNumber() const;

private slots:
    void on_buttonBox_accepted();

    void on_listFieldsWidget_currentRowChanged(int currentRow);

    void on_listFieldsWidget_doubleClicked(const QModelIndex &index);

private:
    Ui::SelectXAxisDialog *ui;
    int _selected_row_number;
};

#endif // SELECTXAXISDIALOG_H
