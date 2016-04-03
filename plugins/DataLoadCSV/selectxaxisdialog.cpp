#include "selectxaxisdialog.h"
#include "ui_selectxaxisdialog.h"
#include <QDebug>

SelectXAxisDialog::SelectXAxisDialog(QStringList *fields, QWidget *parent) :
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

SelectXAxisDialog::~SelectXAxisDialog()
{
    delete ui;
}

int SelectXAxisDialog::getSelectedRowNumber() const
{
    return _selected_row_number;
}

void SelectXAxisDialog::on_buttonBox_accepted()
{
    _selected_row_number = ui->listFieldsWidget->currentRow() - 1;
}

void SelectXAxisDialog::on_listFieldsWidget_currentRowChanged(int )
{
    ui->buttonBox->setEnabled( true );
    _selected_row_number  = ui->listFieldsWidget->currentRow() - 1;
}

void SelectXAxisDialog::on_listFieldsWidget_doubleClicked(const QModelIndex &)
{
    _selected_row_number  = ui->listFieldsWidget->currentRow() - 1;
    this->accept();
}
