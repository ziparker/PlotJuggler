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

    connect(ui->actionSave_layout,SIGNAL(triggered()),      parent, SLOT(onActionSaveLayout()) );
    connect(ui->actionLoad_layout,SIGNAL(triggered()),      parent, SLOT(onActionLoadLayout()) );
    connect(ui->actionLoadData,SIGNAL(triggered()),         parent, SLOT(onActionLoadDataFile()) );
    connect(ui->actionLoad_Recent_file,SIGNAL(triggered()), parent, SLOT(onActionReloadDataFile()) );

    createActions();
}


void FilterableListWidget::createActions()
{
    QMenu* menuFile = new QMenu("File", ui->mainToolBar);
    menuFile->addAction(ui->actionLoadData);

    menuFile->addAction(ui->actionLoad_Recent_file);

    QSettings settings( "IcarusTechnology", "SuperPlotter-0.1");
    if( settings.contains("recentlyLoadedFile") )
    {
        QString fileName = settings.value("recentlyLoadedFile").toString();
        ui->actionLoad_Recent_file->setText( "Load data from: " + fileName);
        ui->actionLoad_Recent_file->setEnabled( true );
    }
    else{
        ui->actionLoad_Recent_file->setEnabled( false );
    }
    menuFile->addSeparator();
    menuFile->addAction(ui->actionLoad_layout);
    menuFile->addAction(ui->actionSave_layout);
    ui->mainToolBar->addAction( menuFile->menuAction());
    ui->verticalLayout->insertWidget(0, ui->mainToolBar );
}


FilterableListWidget::~FilterableListWidget()
{
    delete ui;
}

QListWidget *FilterableListWidget::list()
{
    return ui->listWidget;
}

QAction *FilterableListWidget::actionLoadRecentFile()
{
    return ui->actionLoad_Recent_file;
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
