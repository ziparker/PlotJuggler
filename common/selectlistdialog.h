#ifndef SELECT_FROM_LIST_DIALOG_H
#define SELECT_FROM_LIST_DIALOG_H

#include <QDialog>
#include "ui_selectlistdialog.h"

namespace Ui {
class SelectFromListDialog;
}

class SelectFromListDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SelectFromListDialog(QStringList* fields, bool single_selection = true, QWidget *parent = 0);
    ~SelectFromListDialog();

    std::vector<int> getSelectedRowNumber() const;

private slots:
    void on_buttonBox_accepted();

    void on_listFieldsWidget_currentRowChanged(int currentRow);

    void on_listFieldsWidget_doubleClicked(const QModelIndex &index);

    void on_pushButtonSelectAll_pressed();

private:
    Ui::SelectXAxisDialog *ui;
    std::vector<int> _selected_row_number;
    bool _single_selection;
};


//-----------------------------------------------
inline SelectFromListDialog::SelectFromListDialog(QStringList *fields, bool single_selection, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SelectXAxisDialog),
    _single_selection( single_selection )
{
    auto flags = this->windowFlags();
    this->setWindowFlags( flags | Qt::WindowStaysOnTopHint);

    ui->setupUi(this);

    if( ! _single_selection) {
        ui->listFieldsWidget->setSelectionMode( QAbstractItemView::ExtendedSelection );
    }
    else{
        ui->pushButtonSelectAll->hide();
    }

    for (int i=0; i< fields->size(); i++) {
        ui->listFieldsWidget->addItem( new QListWidgetItem( (*fields)[i] ) );
    }
}

inline SelectFromListDialog::~SelectFromListDialog()
{
    delete ui;
}

inline std::vector<int> SelectFromListDialog::getSelectedRowNumber() const
{
    return _selected_row_number;
}

inline void SelectFromListDialog::on_buttonBox_accepted()
{
    QModelIndexList indexes = ui->listFieldsWidget->selectionModel()->selectedIndexes();

    foreach(QModelIndex index, indexes)
    {
        _selected_row_number.push_back(index.row());
    }
}

inline void SelectFromListDialog::on_listFieldsWidget_currentRowChanged(int )
{
    QModelIndexList indexes = ui->listFieldsWidget->selectionModel()->selectedIndexes();
    ui->buttonBox->setEnabled( indexes.empty() == false );
}

inline void SelectFromListDialog::on_listFieldsWidget_doubleClicked(const QModelIndex &)
{
    if( _single_selection ) {
        _selected_row_number.push_back( ui->listFieldsWidget->currentRow() );
        this->accept();
    }
}

inline void SelectFromListDialog::on_pushButtonSelectAll_pressed()
{
    for (int i=0; i< ui->listFieldsWidget->count(); i++)
    {
        _selected_row_number.push_back(i);
    }
    this->accept();
}

#endif // SELECTXAXISDIALOG_H
