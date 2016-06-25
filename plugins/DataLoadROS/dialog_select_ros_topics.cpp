#include <QFile>
#include <QTextStream>
#include <QSettings>
#include <QFileInfo>
#include <QDir>
#include <QFileDialog>
#include <QListView>
#include <QListWidgetItem>
#include "dialog_select_ros_topics.h"
#include "ui_dialog_select_ros_topics.h"

const char* PREVIOUS_FILE = "previouslyLoadedSubstitutionRules";

DialogSelectRosTopics::DialogSelectRosTopics(QStringList topic_list, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dialogSelectRosTopics)
{
    auto flags = this->windowFlags();
    this->setWindowFlags( flags | Qt::WindowStaysOnTopHint);

    ui->setupUi(this);

    for (int i=0; i< topic_list.size(); i++) {
        ui->listRosTopics->addItem( new QListWidgetItem( topic_list[i] ) );
    }

    QSettings settings( "IcarusTechnology", "SuperPlotter");

    if( settings.contains( PREVIOUS_FILE ) )
    {
        QString fileName = settings.value( PREVIOUS_FILE ).toString();
        // check if it exist
        QFile file(fileName);

        if( file.exists() )
        {
            readRuleFile( file );
        }
    }
}

DialogSelectRosTopics::~DialogSelectRosTopics()
{
    delete ui;
}

QStringList DialogSelectRosTopics::getSelectedItems()
{
    return _topic_list;
}

void DialogSelectRosTopics::readRuleFile(QFile &file)
{
    if ( file.open(QFile::ReadOnly | QFile::Text))
    {
        QTextStream in(&file);
        this->_loaded_rules = in.readAll();

        QSettings settings( "IcarusTechnology", "SuperPlotter");
        settings.setValue( PREVIOUS_FILE, file.fileName() );

        ui->labelLoadedRules->setText( file.fileName() );
        file.close();
    }
}

void DialogSelectRosTopics::on_pushButtonLoadRules_pressed()
{
    QSettings settings( "IcarusTechnology", "SuperPlotter");

    QString directory_path = QDir::currentPath();
    QString fileName;

    if( settings.contains( PREVIOUS_FILE ) )
    {
        fileName = settings.value( PREVIOUS_FILE ).toString();
        QFileInfo info( fileName );
        directory_path = info.absoluteFilePath();
    }

    fileName = QFileDialog::getOpenFileName(this, "Open Layout", directory_path, "*.txt");

    if (fileName.isEmpty())
        return;

    QFile file(fileName);
    readRuleFile( file );
}

void DialogSelectRosTopics::on_buttonBox_accepted()
{
    QModelIndexList indexes = ui->listRosTopics->selectionModel()->selectedIndexes();

    foreach(QModelIndex index, indexes)
    {
        _topic_list.push_back( index.data(Qt::DisplayRole).toString() );
    }
}

void DialogSelectRosTopics::on_listRosTopics_itemSelectionChanged()
{
    QModelIndexList indexes = ui->listRosTopics->selectionModel()->selectedIndexes();

    ui->buttonBox->setEnabled( indexes.size() > 0) ;
}
