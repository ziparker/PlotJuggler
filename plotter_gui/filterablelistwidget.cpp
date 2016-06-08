#include "filterablelistwidget.h"
#include "ui_filterablelistwidget.h"

#include <QLayoutItem>
#include <QMenu>
#include <QSettings>


FilterableListWidget::FilterableListWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FilterableListWidget)
{
    ui->setupUi(this);

    for( int i=0; i< ui->gridLayoutSettings->count(); i++)
    {
        QLayoutItem* item = ui->gridLayoutSettings->itemAt(i);
        if(item)
        {
            QWidget* widget = item->widget();
            if(widget) {
                widget->setMaximumHeight( 0 );
                widget->setVisible( false );
            }
        }
    }
}

FilterableListWidget::~FilterableListWidget()
{
    delete ui;
}

QListWidget *FilterableListWidget::list()
{
    return ui->listWidget;
}


void FilterableListWidget::on_radioContains_toggled(bool checked)
{
    if(checked)
    {
        ui->radioRegExp->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void FilterableListWidget::on_radioRegExp_toggled(bool checked)
{
    if(checked)
    {
        ui->radioContains->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void FilterableListWidget::on_checkBoxCaseSensitive_toggled(bool checked)
{
    on_lineEdit_textChanged( ui->lineEdit->text() );
}

void FilterableListWidget::on_radioTreeView_toggled(bool checked)
{
    if(checked)
    {
        ui->radioFlatView->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void FilterableListWidget::on_radioFlatView_toggled(bool checked)
{
    if(checked)
    {
        ui->radioTreeView->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void FilterableListWidget::on_lineEdit_textChanged(const QString &search_string)
{
    Qt::CaseSensitivity cs = Qt::CaseInsensitive;
    if( ui->checkBoxCaseSensitive->isChecked())
    {
        cs = Qt::CaseSensitive;
    }
    QRegExp regexp( search_string,  cs, QRegExp::Wildcard );
    QRegExpValidator v(regexp, 0);

    for (int i=0; i< ui->listWidget->count(); i++)
    {
        QListWidgetItem* item = ui->listWidget->item(i);
        QString name = item->text();
        int pos = 0;
        bool toHide = false;

        if( ui->radioRegExp->isChecked())
            toHide = v.validate( name, pos ) != QValidator::Acceptable;
        else{
            QStringList items = search_string.split(' ');
            for (int i=0; i< items.size(); i++)
            {
                if( name.contains(items[i], cs) == false )
                {
                    toHide = true;
                }
            }
        }

        item->setHidden( toHide );
    }
}

void FilterableListWidget::on_pushButtonSettings_toggled(bool checked)
{
    for( int i=0; i< ui->gridLayoutSettings->count(); i++)
    {
        QLayoutItem* item = ui->gridLayoutSettings->itemAt(i);
        if(item)
        {
            QWidget* widget = item->widget();
            if(widget)
            {
                widget->setMaximumHeight( checked ? 25:0 );
                widget->setVisible( checked );

            }
        }
    }
}
