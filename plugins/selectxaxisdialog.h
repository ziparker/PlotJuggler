#ifndef SELECTXAXISDIALOG_H
#define SELECTXAXISDIALOG_H

#include <QDialog>
#include "ui_selectxaxisdialog.h"

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


//-----------------------------------------------
inline SelectXAxisDialog::SelectXAxisDialog(QStringList *fields, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SelectXAxisDialog)
{
    ui->setupUi(this);

     ui->listFieldsWidget->addItem( new QListWidgetItem("INDEX (auto-generated)" ) );

    for (int i=0; i< fields->size(); i++)
    {
        ui->listFieldsWidget->addItem( new QListWidgetItem( (*fields)[i] ) );
    }
    _selected_row_number = -1;
}

inline SelectXAxisDialog::~SelectXAxisDialog()
{
    delete ui;
}

inline int SelectXAxisDialog::getSelectedRowNumber() const
{
    return _selected_row_number;
}

inline void SelectXAxisDialog::on_buttonBox_accepted()
{
    _selected_row_number = ui->listFieldsWidget->currentRow() - 1;
}

inline void SelectXAxisDialog::on_listFieldsWidget_currentRowChanged(int )
{
    ui->buttonBox->setEnabled( true );
    _selected_row_number  = ui->listFieldsWidget->currentRow() - 1;
}

inline void SelectXAxisDialog::on_listFieldsWidget_doubleClicked(const QModelIndex &)
{
    _selected_row_number  = ui->listFieldsWidget->currentRow() - 1;
    this->accept();
}

#endif // SELECTXAXISDIALOG_H
