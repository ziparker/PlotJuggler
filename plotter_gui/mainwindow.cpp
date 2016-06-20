#include <functional>
#include <QMouseEvent>
#include <QDebug>
#include <numeric>
#include <QMimeData>
#include <QMenu>
#include <QStringListModel>
#include <stdio.h>
#include <qwt_plot_canvas.h>
#include <QDomDocument>
#include <QFileDialog>
#include <QMessageBox>
#include <QStringRef>
#include <QThread>
#include <QPluginLoader>
#include <QSettings>
#include <QWindow>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "busydialog.h"
#include "busytaskdialog.h"
#include "filterablelistwidget.h"
#include "tabbedplotwidget.h"
#include "subwindow.h"
#include "selectlistdialog.h"

int unique_number = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    QLocale::setDefault(QLocale::c()); // set as default

    curvelist_widget = new FilterableListWidget(this);

    ui->setupUi(this);

    _tabbed_plotarea.push_back( new TabbedPlotWidget( &_mapped_plot_data, this) );
    connect( _tabbed_plotarea.back(), SIGNAL(undoableChangeHappened()), this, SLOT(onUndoableChange()) );

    ui->centralLayout->insertWidget(0, _tabbed_plotarea.back());
    ui->leftLayout->addWidget( curvelist_widget );


    //ui->splitter->insertWidget(0, curvelist_widget);

  //  ui->splitter->setStretchFactor(0,1);
  //  ui->splitter->setStretchFactor(1,1);

    connect( ui->splitter, SIGNAL(splitterMoved(int,int)), SLOT(onSplitterMoved(int,int)) );

    createActions();
    loadPlugins("plugins");

    //  buildData();
    _undo_timer.start();

    // save initial state
    onUndoableChange();

    _replot_timer = new QTimer(this);
    connect(_replot_timer, SIGNAL(timeout()), this, SLOT(onReplotRequested()));

    ui->horizontalSpacer->changeSize(0,0, QSizePolicy::Fixed, QSizePolicy::Fixed);
    ui->streamingLabel->setHidden(true);
    ui->streamingSpinBox->setHidden(true);
    this->repaint();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onUndoableChange()
{
    /* int elapsed_ms = _undo_timer.restart();

    // overwrite the previous
    if( elapsed_ms < 300)
    {
        if( _undo_states.empty() == false)
            _undo_states.pop_back();
    }*/

    _undo_states.push_back( xmlSaveState() );
    updateInternalState();
    _redo_states.clear();
}

void MainWindow::getMaximumRangeX(double* minX, double* maxX)
{
    *minX = std::numeric_limits<double>::max();
    *maxX = std::numeric_limits<double>::min();

    for ( unsigned i = 0; i< _plot_matrix_list.size(); i++ )
    {
        PlotMatrix* matrix = _plot_matrix_list[i];

        for ( unsigned w = 0; w< matrix->plotCount(); w++ )
        {
            PlotWidget *plot =  matrix->plotAt(w);
            QRectF bound_max = plot->maximumBoundingRect();

            if( *minX > bound_max.left() )    *minX = bound_max.left();
            if( *maxX < bound_max.right() )   *maxX = bound_max.right();
        }
    }
}


void MainWindow::onTrackerTimeUpdated(double current_time)
{
    double minX, maxX;
    getMaximumRangeX( &minX, &maxX );

    double ratio = (current_time - minX)/(double)(maxX-minX);

    double min_slider = (double)ui->horizontalSlider->minimum();
    double max_slider = (double)ui->horizontalSlider->maximum();
    int slider_value = (int)((max_slider- min_slider)* ratio) ;

    ui->horizontalSlider->setValue(slider_value);

    for(unsigned i=0; i< _plot_matrix_list.size(); i++)
    {
        PlotMatrix* matrix = _plot_matrix_list[i];

        for ( unsigned w = 0; w< matrix->plotCount(); w++ )
        {
            PlotWidget *plot =  matrix->plotAt(w);
            plot->tracker()->manualMove( QPointF(current_time,0) );
        }
        matrix->replot();
    }

    //------------------------

    for ( int i=0; i<state_publisher.size(); i++)
    {
        state_publisher[i]->updateState( &_mapped_plot_data, current_time);
    }
}

void MainWindow::onTrackerPositionUpdated(QPointF pos)
{
    onTrackerTimeUpdated( pos.x() );
}

void MainWindow::createTabbedDialog(PlotMatrix* first_tab, bool undoable)
{
    SubWindow* window = new SubWindow( this );
    Qt::WindowFlags flags = window->windowFlags();
    window->setWindowFlags( flags | Qt::SubWindow );

    const char prefix[] = "Window ";
    int window_number = 1;

    bool number_taken = true;
    while( number_taken )
    {
        number_taken = false;
        for (int i=0; i< _floating_window.size(); i++ )
        {
            QString win_title = _floating_window.at(i)->windowTitle();
            win_title.remove(0, sizeof(prefix)-1 );
            int num = win_title.toInt();

            if (num == window_number)
            {
                number_taken = true;
                window_number++;
                break;
            }
        }
    }

    window->setWindowTitle( QString(prefix) + QString::number(window_number));

    _floating_window.push_back( window );

    TabbedPlotWidget *tabbed_widget = new TabbedPlotWidget( &_mapped_plot_data, first_tab, this, window);
    _tabbed_plotarea.push_back( tabbed_widget );
    tabbed_widget->setStreamingMode( ui->pushButtonStreaming->isChecked() );

    connect( tabbed_widget, SIGNAL(undoableChangeHappened()), this, SLOT(onUndoableChange()) );
    connect( tabbed_widget, SIGNAL(destroyed(QObject*)), this,  SLOT(on_tabbedAreaDestroyed(QObject*)) );
    connect( window, SIGNAL(destroyed(QObject*)),        this,  SLOT(onFloatingWindowDestroyed(QObject*)) );
    connect( window, SIGNAL(closeRequestedByUser()),     this,  SLOT(onUndoableChange()) );

    window->setCentralWidget( tabbed_widget );
    window->setAttribute( Qt::WA_DeleteOnClose, true );
    window->show();
    window->activateWindow();

    window->addAction( _actionUndo );
    window->addAction( _actionRedo );


    if( undoable ) onUndoableChange();
}


void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    const QMimeData *mimeData = event->mimeData();
    QStringList mimeFormats = mimeData->formats();

    foreach(QString format, mimeFormats)
    {
        qDebug() << " mimestuff " << format;
    }
}

void MainWindow::createActions()
{
    _actionUndo = new QAction(tr("Undo"),this);
    _actionRedo = new QAction(tr("Redo"),this);
    _actionUndo->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_Z));
    _actionRedo->setShortcut( QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_Z));

    this->addAction( _actionUndo );
    this->addAction( _actionRedo );
    connect(_actionUndo, SIGNAL(triggered()), this, SLOT(onUndoInvoked()) );
    connect(_actionRedo, SIGNAL(triggered()), this, SLOT(onRedoInvoked()) );

    //---------------------------------------------

    connect( ui->actionSaveLayout,SIGNAL(triggered()),        this, SLOT(onActionSaveLayout()) );
    connect(ui->actionLoadLayout,SIGNAL(triggered()),         this, SLOT(onActionLoadLayout()) );
    connect(ui->actionLoadData,SIGNAL(triggered()),           this, SLOT(onActionLoadDataFile()) );
    connect(ui->actionLoadRecentDatafile,SIGNAL(triggered()), this, SLOT(onActionReloadDataFileFromSettings()) );
    connect(ui->actionLoadRecentLayout,SIGNAL(triggered()),   this, SLOT(onActionReloadRecentLayout()) );
    connect(ui->actionReloadData,SIGNAL(triggered()),         this, SLOT(onActionReloadSameDataFile()) );
    connect(ui->actionStartStreaming,SIGNAL(triggered()),     this, SLOT(onActionLoadStreamer()) );
    connect(ui->actionDeleteAllData,SIGNAL(triggered()),      this, SLOT(onDeleteLoadedData()) );

    //---------------------------------------------

    QSettings settings( "IcarusTechnology", "SuperPlotter-0.1");
    if( settings.contains("recentlyLoadedDatafile") )
    {
        QString fileName = settings.value("recentlyLoadedDatafile").toString();
        ui->actionLoadRecentDatafile->setText( "Load data from: " + fileName);
        ui->actionLoadRecentDatafile->setEnabled( true );
    }
    else{
        ui->actionLoadRecentDatafile->setEnabled( false );
    }

    ui->actionReloadData->setEnabled( false );
    ui->actionDeleteAllData->setEnabled( false );

    if( settings.contains("recentlyLoadedLayout") )
    {
        QString fileName = settings.value("recentlyLoadedLayout").toString();
        ui->actionLoadRecentLayout->setText( "Load layout from: " + fileName);
        ui->actionLoadRecentLayout->setEnabled( true );
    }
    else{
        ui->actionLoadRecentLayout->setEnabled( false );
    }
}


QColor MainWindow::colorHint()
{
    static int index = 0;
    QColor color;
    switch( index%9 )
    {
    case 0:  color = QColor(Qt::black) ;break;
    case 1:  color = QColor(Qt::blue);break;
    case 2:  color =  QColor(Qt::red); break;
    case 3:  color =  QColor(Qt::darkGreen); break;
    case 4:  color =  QColor(Qt::magenta); break;
    case 5:  color =  QColor(Qt::darkCyan); break;
    case 6:  color =  QColor(Qt::gray); break;
    case 7:  color =  QColor(Qt::darkBlue); break;
    case 8:  color =  QColor(Qt::darkYellow); break;
    }
    index++;
    return color;
}

void MainWindow::loadPlugins(QString subdir_name)
{
    QDir pluginsDir(qApp->applicationDirPath());

#if defined(Q_OS_WIN)
    if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
        pluginsDir.cdUp();
#endif

    pluginsDir.cd( subdir_name );

    foreach (QString fileName, pluginsDir.entryList(QDir::Files))
    {
        QPluginLoader pluginLoader(pluginsDir.absoluteFilePath(fileName));

        QObject *plugin = pluginLoader.instance();
        if (plugin)
        {
            DataLoader *loader = qobject_cast<DataLoader *>(plugin);
            if (loader)
            {
                std::vector<const char*> extensions = loader->compatibleFileExtensions();
                qDebug() << fileName << ": is a DataLoader plugin";

                for(unsigned i = 0; i < extensions.size(); i++)
                {
                    data_loader.insert( std::make_pair( QString(extensions[i]), loader) );
                }
            }

            StatePublisher *publisher = qobject_cast<StatePublisher *>(plugin);
            if (publisher)
            {
                qDebug() << fileName << ": is a StatePublisher plugin";
                state_publisher.push_back( publisher );
            }

            DataStreamer *streamer =  qobject_cast<DataStreamer *>(plugin);
            if (streamer)
            {
                qDebug() << fileName << ": is a DataStreamer plugin";
                data_streamer.push_back( streamer );
            }
        }
        else{
            if( pluginLoader.errorString().contains("is not an ELF object") == false)
            {
                qDebug() << fileName << ": " << pluginLoader.errorString();
            }
        }
    }
}

void MainWindow::buildData()
{
    long SIZE = 100*1000;

    QStringList  words_list;
    words_list << "siam" << "tre" << "piccoli" << "porcellin"
               << "mai" << "nessun" << "ci" << "dividera";

    curvelist_widget->list()->addItems( words_list );


    foreach( const QString& name, words_list)
    {

        float A =  qrand()/(float)RAND_MAX * 6 - 3;
        float B =  qrand()/(float)RAND_MAX *3;
        float C =  qrand()/(float)RAND_MAX *3;
        float D =  qrand()/(float)RAND_MAX *2 -1;

        PlotDataPtr plot ( new PlotData(  ) );
        plot->setName(  name.toStdString() );
        plot->setCapacity( SIZE );

        double t = 0;
        for (int indx=0; indx<SIZE; indx++)
        {
            t += 0.001;
            plot->pushBack( t,  A*sin(B*t + C) +D*t*0.02 ) ;
        }

        QColor color = colorHint();
        plot->setColorHint( color.red(), color.green(), color.blue() );

        _mapped_plot_data.insert( std::make_pair( name.toStdString(), plot) );
    }
    ui->horizontalSlider->setRange(0, SIZE  );

}


void MainWindow::mousePressEvent(QMouseEvent *)
{

}

void MainWindow::onSplitterMoved(int , int )
{
    QList<int> sizes = ui->splitter->sizes();
    int maxLeftWidth = curvelist_widget->list()->maximumWidth();
    int totalWidth = sizes[0] + sizes[1];

    if( sizes[0] > maxLeftWidth)
    {
        sizes[0] = maxLeftWidth;
        sizes[1] = totalWidth - maxLeftWidth;
        ui->splitter->setSizes(sizes);
    }
}

void MainWindow::resizeEvent(QResizeEvent *)
{
    onSplitterMoved( 0, 0 );
}


void MainWindow::onPlotAdded(PlotWidget* plot)
{
    connect( plot, SIGNAL(plotModified()),         this, SLOT(onUndoableChange()) );
    connect( plot, SIGNAL(trackerMoved(QPointF)),  this, SLOT(onTrackerPositionUpdated(QPointF)));
    connect( plot, SIGNAL(swapWidgets(PlotWidget*,PlotWidget*)), this, SLOT(onSwapPlots(PlotWidget*,PlotWidget*)) );
}


QDomDocument MainWindow::xmlSaveState()
{
    QDomDocument doc;
    QDomProcessingInstruction instr = doc.createProcessingInstruction(
                "xml", "version='1.0' encoding='UTF-8'");

    doc.appendChild(instr);

    QDomElement root = doc.createElement( "root" );

    for (int i = 0; i < _tabbed_plotarea.size(); i++)
    {
        QDomElement tabbed_area = _tabbed_plotarea[i]->xmlSaveState(doc);
        root.appendChild( tabbed_area );
    }

    doc.appendChild(root);

    return doc;
}

bool MainWindow::xmlLoadState(QDomDocument state_document)
{
    QDomElement root = state_document.namedItem("root").toElement();
    if ( root.isNull() ) {
        qWarning() << "No <root> element found at the top-level of the XML file!";
        return false;
    }

    QDomElement tabbed_area;

    int num_floating = 0;

    for (  tabbed_area = root.firstChildElement(  "tabbed_widget" )  ;
           tabbed_area.isNull() == false;
           tabbed_area = tabbed_area.nextSiblingElement( "tabbed_widget" ) )
    {
        if( tabbed_area.attribute("parent").compare("main_window") != 0)
        {
            num_floating++;
        }
    }

    // add windows if needed
    while( _floating_window.size() < num_floating )
    {
        createTabbedDialog( NULL, false );
    }

    while( _floating_window.size() > num_floating ){
        QMainWindow* window =  _floating_window.back();
        _floating_window.pop_back();
        window->deleteLater();
    }

    //-----------------------------------------------------

    int index = 1;

    for (  tabbed_area = root.firstChildElement(  "tabbed_widget" )  ;
           tabbed_area.isNull() == false;
           tabbed_area = tabbed_area.nextSiblingElement( "tabbed_widget" ) )
    {


        if( tabbed_area.attribute("parent").compare("main_window") == 0)
        {
            _tabbed_plotarea.front()->xmlLoadState( tabbed_area );
        }
        else{

            TabbedPlotWidget* tabbed_widget = _tabbed_plotarea.at( index++ );
            tabbed_widget->xmlLoadState( tabbed_area );
        }
    }

    return true;
}

void MainWindow::onActionSaveLayout()
{
    QDomDocument doc = xmlSaveState();

    if( _loaded_datafile.isEmpty() == false)
    {
        QDomElement root = doc.namedItem("root").toElement();
        QDomElement previously_loaded_datafile =  doc.createElement( "previouslyLoadedDatafile" );
        QDomText textNode = doc.createTextNode( _loaded_datafile );

        previously_loaded_datafile.appendChild( textNode );
        root.appendChild( previously_loaded_datafile );
    }

    QSettings settings( "IcarusTechnology", "SuperPlotter-0.1");

    QString directory_path = QDir::currentPath();

    if( settings.contains("lastLayoutDirectory") )
    {
        directory_path = settings.value("lastLayoutDirectory").toString();
    }

    QString filename = QFileDialog::getSaveFileName(this, "Save Layout", directory_path, "*.xml");
    if (filename.isEmpty())
        return;

    if(filename.endsWith(".xml",Qt::CaseInsensitive) == false)
    {
        filename.append(".xml");
    }

    QFile file(filename);
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream stream(&file);
        stream << doc.toString() << endl;
    }
}

void MainWindow::deleteLoadedData(const QString& curve_name)
{
    auto plot_data = _mapped_plot_data.find( curve_name.toStdString() );
    if( plot_data == _mapped_plot_data.end())
    {
        return;
    }

    auto items_to_remove = curvelist_widget->list()->findItems( curve_name, Qt::MatchExactly );
    qDeleteAll( items_to_remove );

    for (int i = 0; i < _tabbed_plotarea.size(); i++)
    {
        QTabWidget* tab_widget = _tabbed_plotarea[i]->tabWidget();
        for (int t = 0; t < tab_widget->count(); t++)
        {
            PlotMatrix* tab = static_cast<PlotMatrix*>( tab_widget->widget(t) );
            if (tab){
                for (int p=0; p < tab->plotCount(); p++)
                {
                    PlotWidget* plot = tab->plotAt(p);
                    plot->removeCurve( curve_name );
                }
            }
        }
    }
    _mapped_plot_data.erase( plot_data );

    if( curvelist_widget->list()->count() == 0)
    {
        ui->actionReloadData->setEnabled( false );
        ui->actionDeleteAllData->setEnabled( false );
    }
}


void MainWindow::onDeleteLoadedData()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(0, tr("Warning"),
                                  tr("Do you really want to remove any loaded data?\n"),
                                  QMessageBox::Yes | QMessageBox::No,
                                  QMessageBox::No );
    if( reply == QMessageBox::No ) {
        return;
    }

    _mapped_plot_data.erase( _mapped_plot_data.begin(),
                             _mapped_plot_data.end() );

    curvelist_widget->list()->clear();

    for (int i = 0; i < _tabbed_plotarea.size(); i++)
    {
        QTabWidget* tab_widget = _tabbed_plotarea[i]->tabWidget();
        for (int t = 0; t < tab_widget->count(); t++)
        {
            PlotMatrix* tab = static_cast<PlotMatrix*>( tab_widget->widget(t) );
            if (tab){
                tab->removeAllCurves();
            }
        }
    }
    ui->actionReloadData->setEnabled( false );
    ui->actionDeleteAllData->setEnabled( false );
}

void MainWindow::onActionLoadDataFile(bool reload_from_settings)
{
    if( data_loader.empty())
    {
        QMessageBox::warning(0, tr("Warning"),
                             tr("No plugin was loaded to process a data file\n") );
        return;
    }

    QSettings settings( "IcarusTechnology", "SuperPlotter-0.1");

    std::map<QString,DataLoader*>::iterator it;

    QString file_extension_filter;

    for (it = data_loader.begin(); it != data_loader.end(); it++)
    {
        QString extension = it->first.toLower();
        file_extension_filter.append( QString(" *.") + extension );
    }

    QString directory_path = QDir::currentPath();

    const QString SETTINGS_KEY( "lastDatafileDirectory");

    if( settings.contains(SETTINGS_KEY) )
    {
        directory_path = settings.value(SETTINGS_KEY).toString();
    }

    QString fileName;
    if( reload_from_settings && settings.contains("recentlyLoadedDatafile") )
    {
        fileName = settings.value("recentlyLoadedDatafile").toString();
    }
    else{
        fileName = QFileDialog::getOpenFileName(this,
                                                "Open Datafile",
                                                directory_path,
                                                file_extension_filter);
    }

    if (fileName.isEmpty())
        return;

    directory_path = QFileInfo(fileName).absolutePath();

    settings.setValue(SETTINGS_KEY, directory_path);
    settings.setValue("recentlyLoadedDatafile", fileName);

    ui->actionLoadRecentDatafile->setText("Load data from: " + fileName);

    onActionLoadDataFile( fileName );
}

void MainWindow::onActionLoadDataFile(QString fileName)
{
    DataLoader* loader = data_loader[ QFileInfo(fileName).suffix() ];

    if( loader )
    {
        QFile file(fileName);

        if (!file.open(QFile::ReadOnly | QFile::Text)) {
            QMessageBox::warning(this, tr("Datafile"),
                                 tr("Cannot read file %1:\n%2.")
                                 .arg(fileName)
                                 .arg(file.errorString()));
            return;
        }

        _loaded_datafile = fileName;
        ui->actionReloadData->setEnabled( true );
        ui->actionDeleteAllData->setEnabled( true );

        BusyTaskDialog* busy = new BusyTaskDialog("Loading file");
        busy->show();
        using namespace std::placeholders;

        int last_timeindex = TIME_INDEX_NOT_DEFINED;

        PlotDataMap mapped_data = loader->readDataFromFile( &file,
                                                            [busy](int value) {
            busy->setValue(value);
            QApplication::processEvents();
        },
        [busy]() { return busy->wasCanceled(); },
        last_timeindex);

        busy->close();

        // remap to different type
        PlotDataMap::iterator it;

        size_t maxSizeX = 0;

        for ( it = mapped_data.begin(); it != mapped_data.end(); it++)
        {
            std::string name  = it->first;
            PlotDataPtr plot  = it->second;

            if( maxSizeX <  plot->size() ){
                maxSizeX =  plot->size();
            }

            QString qname = QString::fromStdString(name);

            // remap to derived class
            if( _mapped_plot_data.find(name) == _mapped_plot_data.end() )
            {
                _mapped_plot_data.insert( std::make_pair( name, plot) );

                QColor color = colorHint();
                plot->setColorHint( color.red(), color.green(), color.blue() );
                curvelist_widget->list()->addItem( new QListWidgetItem( qname ) );
            }
            else{
                // update plot if it was already loaded
                _mapped_plot_data[name] = plot;
            }
        }

        if( _mapped_plot_data.size() > mapped_data.size() )
        {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(0, tr("Warning"),
                                          tr("Do you want to remove the previously loaded data?\n"),
                                          QMessageBox::Yes | QMessageBox::No,
                                          QMessageBox::Yes );
            if( reply == QMessageBox::Yes )
            {
                bool repeat = true;
                while( repeat )
                {
                    repeat = false;
                    for ( it = _mapped_plot_data.begin(); it != _mapped_plot_data.end(); it++ )
                    {
                        auto& name = it->first;
                        if( mapped_data.find( name ) == mapped_data.end() )
                        {
                            qDebug() << " delete "  <<  QString( name.c_str() );
                            this->deleteLoadedData( QString( name.c_str() ) );
                            repeat = true;
                            break;
                        }
                    }
                }
            }
        }

        _undo_states.clear();
        _redo_states.clear();

        _undo_states.push_back(  xmlSaveState() );
        ui->horizontalSlider->setRange(0, maxSizeX );

        updateInternalState();
    }
    else{
        QMessageBox::warning(this, tr("Error"),
                             tr("Cannot read files with extension %1.\n No plugin can handle that!\n")
                             .arg(fileName) );
    }
}

void MainWindow::onActionReloadSameDataFile()
{
    onActionLoadDataFile(_loaded_datafile);
}

void MainWindow::onActionReloadDataFileFromSettings()
{
    onActionLoadDataFile( true );
}

void MainWindow::onActionReloadRecentLayout()
{
    onActionLoadLayout( true );
}

void MainWindow::onActionLoadStreamer()
{
    if( data_streamer.empty())
    {
        qDebug() << "Error, no streamer loaded";
        return;
    }
    DataStreamer* streamer = data_streamer[0];

    if( data_streamer.size() > 1)
    {
        QStringList streamers_name;
        for (int i=0; i< data_streamer.size(); i++)
        {
            streamers_name.push_back( QString( data_streamer[i]->name()) );
        }
        SelectFromListDialog dialog( &streamers_name, true, this );
        dialog.exec();
        int index = dialog.getSelectedRowNumber().at(0) ;
        if( index >= 0)
        {
            streamer = data_streamer[index];
        }
    }

    streamer->enableStreaming( false );
    //  ui->pushButtonStreaming->hide(false);
    ui->pushButtonStreaming->setEnabled(true);

    PlotDataMap& plot_data = streamer->getDataMap();

    for (auto it = plot_data.begin(); it != plot_data.end(); it++)
    {
        std::string name  = it->first;
        PlotDataPtr plot  = it->second;

        QString qname = QString::fromStdString(name);

        // remap to derived class
        if( _mapped_plot_data.find(name) == _mapped_plot_data.end() )
        {
            _mapped_plot_data.insert( std::make_pair( name, plot) );

            QColor color = colorHint();
            plot->setColorHint( color.red(), color.green(), color.blue() );
            curvelist_widget->list()->addItem( new QListWidgetItem( qname ) );
        }
        else{
            // update plot if it was already loaded
            _mapped_plot_data[name] = plot;
        }
    }
}

void MainWindow::onActionLoadLayout(bool reload_previous)
{
    QSettings settings( "IcarusTechnology", "SuperPlotter-0.1");

    QString directory_path = QDir::currentPath();
    const QString SETTINGS_KEY( "lastLayoutDirectory");

    if( settings.contains(SETTINGS_KEY) )
    {
        directory_path = settings.value(SETTINGS_KEY).toString();
    }

    QString fileName;
    if( reload_previous && settings.contains("recentlyLoadedLayout") )
    {
        fileName = settings.value("recentlyLoadedLayout").toString();
    }
    else{
        fileName = QFileDialog::getOpenFileName(this,
                                                "Open Layout",
                                                directory_path,
                                                "*.xml");
    }

    if (fileName.isEmpty())
        return;

    directory_path = QFileInfo(fileName).absolutePath();
    settings.setValue(SETTINGS_KEY, directory_path);
    settings.setValue("recentlyLoadedLayout", fileName);

    ui->actionLoadRecentLayout->setText("Load layout from: " + fileName);

    QFile file(fileName);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, tr("Layout"),
                             tr("Cannot read file %1:\n%2.")
                             .arg(fileName)
                             .arg(file.errorString()));
        return;
    }

    QString errorStr;
    int errorLine, errorColumn;

    QDomDocument domDocument;

    if (!domDocument.setContent(&file, true, &errorStr, &errorLine, &errorColumn)) {
        QMessageBox::information(window(), tr("XML Layout"),
                                 tr("Parse error at line %1:\n%2")
                                 .arg(errorLine)
                                 .arg(errorStr));
        return;
    }

    QDomElement root = domDocument.namedItem("root").toElement();
    QDomElement previously_loaded_datafile =  root.firstChildElement( "previouslyLoadedDatafile" );

    if( previously_loaded_datafile.isNull() == false)
    {
        QString fileName = previously_loaded_datafile.text();

        QMessageBox::StandardButton reload_previous;
        reload_previous = QMessageBox::question(0, tr("Wait!"),
                                                tr("Do you want to reload the datafile?\n\n[%1]\n").arg(fileName),
                                                QMessageBox::Yes | QMessageBox::No,
                                                QMessageBox::Yes );

        if( reload_previous == QMessageBox::Yes )
        {
            onActionLoadDataFile( fileName );
        }
    }
    ///--------------------------------------------------

    xmlLoadState( domDocument );

    _undo_states.clear();
    _undo_states.push_back( domDocument );

    updateInternalState();
}


void MainWindow::onPushButtonActivateTracker_toggled(bool checked)
{
    for (int i = 0; i < _plot_matrix_list.size(); i++)
    {
        _plot_matrix_list[i]->setActiveTracker( checked );
    }
}

void MainWindow::onUndoInvoked( )
{
    // qDebug() << "on_UndoInvoked "<<_undo_states.size();

    if( _undo_states.size() > 1)
    {
        QDomDocument state_document = _undo_states.back();
        _redo_states.push_back( state_document );
        _undo_states.pop_back();
        state_document = _undo_states.back();
        xmlLoadState( state_document );

        updateInternalState();
    }
}

void MainWindow::onRedoInvoked()
{
    if( _redo_states.size() > 0)
    {
        QDomDocument state_document = _redo_states.back();
        _undo_states.push_back( state_document );
        _redo_states.pop_back();

        xmlLoadState( state_document );

        updateInternalState();
    }
}


void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    QSlider* slider = ui->horizontalSlider;
    double ratio = (double)position / (double)(slider->maximum() -  slider->minimum() );

    double minX, maxX;
    getMaximumRangeX( &minX, &maxX);

    double posX = (maxX-minX) * ratio + minX;
    onTrackerTimeUpdated( posX );
}

void MainWindow::on_tabbedAreaDestroyed(QObject *object)
{
    for (int i=0; i< _tabbed_plotarea.size(); i++)
    {
        if( _tabbed_plotarea[i] == object)
        {
            _tabbed_plotarea.erase( _tabbed_plotarea.begin() + i);
            break;
        }
    }
    updateInternalState();

    this->setFocus();
}

void MainWindow::onFloatingWindowDestroyed(QObject *object)
{
    for (int i=0; i< _floating_window.size(); i++)
    {
        if( _floating_window[i] == object)
        {
            _floating_window.erase( _floating_window.begin() + i);
            break;
        }
    }

    updateInternalState();
}

void MainWindow::onCreateFloatingWindow(PlotMatrix* first_tab)
{
    createTabbedDialog( first_tab, true );
}


void MainWindow::updateInternalState()
{
    // Update the _plot_matrix_list
    _plot_matrix_list.clear();

    for (int i = 0; i < _tabbed_plotarea.size(); i++)
    {
        QTabWidget* tab_widget = _tabbed_plotarea[i]->tabWidget();
        for (int t = 0; t < tab_widget->count(); t++)
        {
            PlotMatrix* tab = static_cast<PlotMatrix*>( tab_widget->widget(t) );
            if (tab){
                _plot_matrix_list.push_back( tab );
            }
        }
    }
    //-----------------------------------------
    //
    std::map<QString,TabbedPlotWidget*> tabbed_map;

    tabbed_map.insert( std::make_pair( QString("Main window"), _tabbed_plotarea[0] ) );

    for (int i = 1; i < _tabbed_plotarea.size(); i++)
    {
        tabbed_map.insert( std::make_pair( _floating_window[i-1]->windowTitle(), _tabbed_plotarea[i] ) );
    }
    for (int i = 0; i < _tabbed_plotarea.size(); i++)
    {
        _tabbed_plotarea[i]->setSiblingsList( tabbed_map );
    }
}

void MainWindow::onPushButtonAddSubwindow_pressed()
{
    createTabbedDialog( NULL, true );
}

void MainWindow::onSwapPlots(PlotWidget *source, PlotWidget *destination)
{
    PlotMatrix* src_matrix = NULL;
    PlotMatrix* dst_matrix = NULL;
    QPoint src_pos;
    QPoint dst_pos;

    //qDebug() << source->windowTitle() << " -> " << destination->windowTitle();

    for(int w=0; w < _tabbed_plotarea.size(); w++)
    {
        QTabWidget * tabs = _tabbed_plotarea[w]->tabWidget();

        for (int t=0; t < tabs->count(); t++)
        {
            PlotMatrix* matrix =  static_cast<PlotMatrix*>(tabs->widget(t));

            for(int row=0; row< matrix->rowsCount(); row++)
            {
                for(int col=0; col< matrix->colsCount(); col++)
                {
                    PlotWidget* plot = matrix->plotAt(row, col);

                    if( plot == source ) {
                        src_matrix = matrix;
                        src_pos.setX( row );
                        src_pos.setY( col );
                    }
                    else if( plot == destination )
                    {
                        dst_matrix = matrix;
                        dst_pos.setX( row );
                        dst_pos.setY( col );
                    }
                }
            }
        }
    }

    src_matrix->gridLayout()->removeWidget( source );
    dst_matrix->gridLayout()->removeWidget( destination );

    src_matrix->gridLayout()->addWidget( destination, src_pos.x(), src_pos.y() );
    dst_matrix->gridLayout()->addWidget( source, dst_pos.x(), dst_pos.y() );

}

void MainWindow::onPushButtonStreaming_toggled(bool checked)
{
    if( checked )
        ui->pushButtonStreaming->setText("Streaming ON");
    else
        ui->pushButtonStreaming->setText("Streaming OFF");

    //TODO fixme
    auto streamer = this->data_streamer[0];
    streamer->enableStreaming( checked ) ;
    _replot_timer->setSingleShot(true);
    _replot_timer->start( 5 );

    if ( !checked )
        ui->horizontalSpacer->changeSize(0,0, QSizePolicy::Fixed, QSizePolicy::Fixed);
    else
        ui->horizontalSpacer->changeSize(1,1, QSizePolicy::Expanding, QSizePolicy::Fixed);

    ui->streamingLabel->setHidden( !checked );
    ui->streamingSpinBox->setHidden( !checked );
    ui->horizontalSlider->setHidden( checked );

    this->repaint();

    for(unsigned i=0; i< _tabbed_plotarea.size(); i++)
    {
        TabbedPlotWidget* area = _tabbed_plotarea[i];
        area->setStreamingMode( checked );
    }
}

void MainWindow::onReplotRequested()
{
    for(unsigned i=0; i< _tabbed_plotarea.size(); i++)
    {
        TabbedPlotWidget* area = _tabbed_plotarea[i];
        PlotMatrix* matrix =  area->currentTab() ;
        matrix->maximumZoomOut();
    }

    if( ui->pushButtonStreaming->isChecked())
    {
        _replot_timer->setSingleShot(true);
        _replot_timer->stop( );
        _replot_timer->start( 40 );
    }
}

void MainWindow::on_streamingSpinBox_valueChanged(int value)
{
    for (auto it = _mapped_plot_data.begin(); it != _mapped_plot_data.end(); it++ )
    {
        auto plot = it->second;
        plot->setMaximumRangeX( value );
    }
}
