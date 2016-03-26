#include "selectxaxisdialog.h"
#include "ui_selectxaxisdialog.h"
#include <QDebug>

selectXAxisDialog::selectXAxisDialog( QVector<QStringRef> fields, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::selectXAxisDialog)
{
    ui->setupUi(this);

    for (int i=0; i< fields.size(); i++)
    {
        ui->listFieldsWidget->addItem( new QListWidgetItem( fields[i].toString() ) );
    }
}

selectXAxisDialog::~selectXAxisDialog()
{
    delete ui;
}

void selectXAxisDialog::on_buttonBox_accepted()
{
    this->setResult( ui->listFieldsWidget->currentRow() );
}

void selectXAxisDialog::on_listFieldsWidget_currentRowChanged(int )
{
    ui->buttonBox->setEnabled( true );
}

void selectXAxisDialog::on_listFieldsWidget_doubleClicked(const QModelIndex &)
{
    this->setResult( ui->listFieldsWidget->currentRow() );
    this->accept();
}
