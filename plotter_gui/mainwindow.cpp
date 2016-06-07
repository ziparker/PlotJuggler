#include <functional>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMouseEvent>
#include <QDebug>
#include <QDrag>
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
#include "../plugins/dataloader_base.h"
#include "../plugins/statepublisher_base.h"
#include <QPluginLoader>
#include "busydialog.h"
#include "busytaskdialog.h"
#include "filterablelistwidget.h"
#include <QSettings>
#include <QLineEdit>
#include <QInputDialog>

QStringList  words_list;
int unique_number = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    words_list << "siam" << "tre" << "piccoli" << "porcellin"
               << "mai" << "nessun" << "ci" << "dividera";

    curvelist_widget = new FilterableListWidget(this);

    ui->splitter->insertWidget(0, curvelist_widget);

    ui->splitter->setStretchFactor(0,0);
    ui->splitter->setStretchFactor(1,1);

    _horizontal_link = true;

    on_addTabButton_pressed();

    ui->pushHorizontalResize->setChecked( _horizontal_link );
    currentPlotGrid()->setHorizontalLink( _horizontal_link );

    this->addAction( ui->actionUndo );
    connect(ui->actionUndo, SIGNAL(triggered(bool)), this, SLOT(on_pushButtonUndo_clicked(bool)) );

    this->addAction( ui->actionRedo );
    connect(ui->actionRedo, SIGNAL(triggered(bool)), this, SLOT(on_pushButtonRedo_clicked(bool)) );

    createActions();
    loadPlugins("plugins");

    buildData();
    _undo_timer.start();

    qApp->installEventFilter( this );
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::undoableChangeHappened()
{
    int elapsed_ms = _undo_timer.restart();

    // overwrite the previous
    if( elapsed_ms < 150)
    {
        if( _undo_states.empty() == false)
            _undo_states.pop_back();
    }
    _undo_states.push_back( xmlSaveState() );
    _redo_states.clear();

}

void MainWindow::onTrackerTimeUpdated(double current_time)
{
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();

    for ( unsigned i = 0; i< currentPlotGrid()->widgetList().size(); i++ )
    {
        PlotWidget *plot =  currentPlotGrid()->widgetList().at(i);
        QRectF bound_max = plot->maximumBoundingRect();
        if( minX > bound_max.left() )    minX = bound_max.left();
        if( maxX < bound_max.right() )   maxX = bound_max.right();
    }

    double ratio = current_time/(double)(maxX-minX);

    double min_slider = (double)ui->horizontalSlider->minimum();
    double max_slider = (double)ui->horizontalSlider->maximum();
    int slider_value = (int)((max_slider- min_slider)* ratio);

    ui->horizontalSlider->setValue(slider_value);

    //------------------------
    qDebug() << "updating " << current_time ;

    for (unsigned i=0; i< state_publisher.size(); i++)
    {
        state_publisher[i]->updateState( &_mapped_plot_data, current_time);
    }


}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{

    if( event->type() == QEvent::MouseButtonDblClick) {

        qDebug() << obj;
        qDebug() << ui->tabWidget->tabBar() << "\n";

        if (obj == ui->tabWidget->tabBar() ) {

            // query and set tab(s) names
            QTabBar *tab_bar = qobject_cast<QTabBar *>(obj);
            if(tab_bar)
            {
                int idx = tab_bar->currentIndex ();

                bool ok = true;
                QString newName = QInputDialog::getText (
                            this, tr ("Change Name of the selected tab"),
                            tr ("Insert New Tab Name"),
                            QLineEdit::Normal,
                            tab_bar->tabText (idx),
                            &ok);

                if (ok) {
                    tab_bar->setTabText (idx, newName);
                }
            }
        }
    }

    // Standard event processing
    return QObject::eventFilter(obj, event);
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

void MainWindow::dragMoveEvent(QDragMoveEvent *)
{

}

void MainWindow::dropEvent(QDropEvent *)
{

}


void MainWindow::createActions()
{
    /* QMenu* menuFile = new QMenu("File", ui->mainToolBar);
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
    ui->leftLayout->insertWidget(0, ui->mainToolBar );*/
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
        qDebug() << fileName;
        QPluginLoader pluginLoader(pluginsDir.absoluteFilePath(fileName));

        QObject *plugin = pluginLoader.instance();
        if (plugin)
        {
            DataLoader *loader = qobject_cast<DataLoader *>(plugin);
            if (loader)
            {
                std::vector<const char*> extensions = loader->compatibleFileExtensions();
                qDebug() << "loaded a DataLoader plugin";

                for(unsigned i = 0; i < extensions.size(); i++)
                {
                    data_loader.insert( std::make_pair( QString(extensions[i]), loader) );
                }
            }

            StatePublisher *publisher = qobject_cast<StatePublisher *>(plugin);
            if (publisher)
            {
                qDebug() << "loaded a StatePublisher plugin";
                state_publisher.push_back( publisher );
            }
        }
    }
}

void MainWindow::buildData()
{
    long SIZE = 100*1000;

    curvelist_widget->list()->addItems( words_list );
    SharedVector t_vector ( new std::vector<double>());
    t_vector->reserve(SIZE);

    double t = 0;
    for (int indx=0; indx<SIZE; indx++)
    {
        t_vector->push_back( t );
        t += 0.001;
    }

    foreach( const QString& name, words_list)
    {
        SharedVector y_vector( new std::vector<double>() );
        y_vector->reserve(SIZE);

        float A =  qrand()/(float)RAND_MAX * 6 - 3;
        float B =  qrand()/(float)RAND_MAX *3;
        float C =  qrand()/(float)RAND_MAX *3;
        float D =  qrand()/(float)RAND_MAX *2 -1;

        for (int indx=0; indx<SIZE; indx++)
        {
            double x = t_vector->at(indx);
            y_vector->push_back(  A*sin(B*x + C) +D*x*0.02 ) ;
        }

        PlotDataPtr plot ( new PlotData(t_vector, y_vector, name.toStdString() ) );

        QColor color = colorHint();
        plot->setColorHint( color.red(), color.green(), color.blue() );

        _mapped_plot_data.insert( std::make_pair( name.toStdString(), plot) );
    }

    ui->horizontalSlider->setRange(0, SIZE  );

}


void MainWindow::mousePressEvent(QMouseEvent *)
{

}

void MainWindow::on_splitter_splitterMoved(int , int )
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
    on_splitter_splitterMoved( 0, 0 );
}

void MainWindow::on_pushAddRow_pressed()
{
    currentPlotGrid()->addRow();
    undoableChangeHappened();
}


void MainWindow::on_pushAddColumn_pressed()
{
    currentPlotGrid()->addColumn();
    undoableChangeHappened();
}


void MainWindow::on_pushremoveEmpty_pressed()
{
    PlotMatrix *grid = currentPlotGrid();
    qDebug() << "-------------";

    for( int row = 0; row< grid->numRows(); row++)
    {
        while( grid->isRowEmpty( row ) && row < grid->numRows() ){
            qDebug() << "remove row";
            grid->removeRow( row );
        }
    }

    for( int col = 0; col< grid->numColumns(); col++)
    {
        while( grid->isColumnEmpty( col ) && col < grid->numColumns() ){
            grid->removeColumn( col );
        }
    }

    if( grid->numColumns() == 0 &&  grid->numRows() == 0 )
    {
        on_pushAddColumn_pressed();
    }
    undoableChangeHappened();
}


void MainWindow::on_plotAdded(PlotWidget* widget)
{
    connect(widget,SIGNAL(plotModified()), SLOT(undoableChangeHappened()) );
    connect( widget->tracker(), SIGNAL(timePointMoved(double)), this, SLOT( onTrackerTimeUpdated( double )) );
}


PlotMatrix *MainWindow::currentPlotGrid()
{
    return static_cast<PlotMatrix*>( ui->tabWidget->currentWidget() );
}

void MainWindow::on_addTabButton_pressed()
{
    PlotMatrix* grid = new PlotMatrix(&_mapped_plot_data, this);
    ui->tabWidget->addTab( grid, QString("plot") );
    connect( grid, SIGNAL(plotAdded(PlotWidget*)), this, SLOT(on_plotAdded(PlotWidget*)));

    connect( grid, SIGNAL(layoutModified()), this, SLOT( undoableChangeHappened()) );

    ui->tabWidget->setCurrentWidget( grid );
    grid->setHorizontalLink( _horizontal_link );

    on_pushAddColumn_pressed();

    undoableChangeHappened();
}

QDomDocument MainWindow::xmlSaveState()
{
    QDomDocument doc;
    QDomProcessingInstruction instr = doc.createProcessingInstruction(
                "xml", "version='1.0' encoding='UTF-8'");

    doc.appendChild(instr);

    QDomElement root = doc.createElement( "root" );

    for(int i=0; i< ui->tabWidget->count(); i++)
    {
        PlotMatrix* widget = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );
        QDomElement element = widget->xmlSaveState(doc);

        // add tab name
        element.setAttribute("tab_name",  ui->tabWidget->tabText(i) );

        root.appendChild( element );
    }

    QDomElement current_plotmatrix =  doc.createElement( "currentPlotMatrix" );
    current_plotmatrix.setAttribute( "index", ui->tabWidget->currentIndex() );
    root.appendChild( current_plotmatrix );

    doc.appendChild(root);

    return doc;
}

void MainWindow::xmlLoadState(QDomDocument state_document)
{
    QDomElement root = state_document.namedItem("root").toElement();
    if ( root.isNull() ) {
        qWarning() << "No <root> element found at the top-level of the XML file!";
        return ;
    }

    int num_tabs = ui->tabWidget->count();
    int index = 0;

    QDomElement plotmatrix_el;

    for (  plotmatrix_el = root.firstChildElement( "plotmatrix" )  ;
           !plotmatrix_el.isNull();
           plotmatrix_el = plotmatrix_el.nextSiblingElement( "plotmatrix" ) )
    {
        // add if tabs are too few
        if( index == num_tabs) {
            on_addTabButton_pressed();
            num_tabs++;
        }
        PlotMatrix* plot_matrix = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
        plot_matrix->xmlLoadState( plotmatrix_el );

        // read tab name
        if( plotmatrix_el.hasAttribute("tab_name"))
        {
            ui->tabWidget->setTabText( index, plotmatrix_el.attribute("tab_name" ) );
        }

        index++;
    }

    // remove if tabs are too much
    while( num_tabs > index ){
        ui->tabWidget->removeTab( num_tabs-1 );
        num_tabs--;
    }

    QDomElement current_plotmatrix =  root.firstChildElement( "currentPlotMatrix" );
    int current_index = current_plotmatrix.attribute( "index" ).toInt();
    ui->tabWidget->setCurrentIndex( current_index );

    currentPlotGrid()->replot();
}

void MainWindow::onActionSaveLayout()
{
    QDomDocument doc = xmlSaveState();

    QString filename = QFileDialog::getSaveFileName(this, "Save Layout", QDir::currentPath(), "*.xml");
    if (filename.isEmpty())
        return;

    if(filename.endsWith(".xml",Qt::CaseInsensitive) == false)
    {
        filename.append(".xml");
    }

    QFile file(filename);
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);
        stream << doc.toString() << endl;
    }
}



void MainWindow::deleteLoadedData()
{
    _mapped_plot_data.erase( _mapped_plot_data.begin(),
                             _mapped_plot_data.end() );

    curvelist_widget->list()->clear();

    for (int index = 0; index < ui->tabWidget->count(); index++)
    {
        PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
        if (tab){
            for (unsigned p = 0; p < tab->widgetList().size(); p++)
            {
                PlotWidget* plot = tab->widgetList().at(p);
                plot->detachAllCurves();
            }
        }
    }
}

void MainWindow::onActionLoadDataFile(bool reload_previous)
{
    if( data_loader.empty())
    {
        QMessageBox::warning(0, tr("Warning"),
                             tr("No plugin was loaded to process a data file\n") );
        return;
    }

    if( _mapped_plot_data.empty() == false)
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(0, tr("Warning"),
                                      tr("Do you want to delete the previously loaded data?\n") );
        if( reply == QMessageBox::Yes )
        {
            deleteLoadedData();
        }
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

    const QString SETTINGS_KEY( "load_directory");

    if( settings.contains(SETTINGS_KEY) )
    {
        directory_path = settings.value(SETTINGS_KEY).toString();
    }

    QString fileName;
    if( reload_previous && settings.contains("recentlyLoadedFile") )
    {
        fileName = settings.value("recentlyLoadedFile").toString();
    }
    else{
        fileName = QFileDialog::getOpenFileName(this, "Open Layout",  directory_path, file_extension_filter);
    }

    if (fileName.isEmpty())
        return;

    directory_path = QFileInfo(fileName).absolutePath();

    settings.setValue(SETTINGS_KEY, directory_path);
    settings.setValue("recentlyLoadedFile", fileName);

    curvelist_widget->actionLoadRecentFile()->setText("Load data from: " + fileName);

    DataLoader* loader = data_loader[ QFileInfo(fileName).suffix() ];

    if( loader )
    {
        QFile file(fileName);

        if (!file.open(QFile::ReadOnly | QFile::Text)) {
            QMessageBox::warning(this, tr("Layout"),
                                 tr("Cannot read file %1:\n%2.")
                                 .arg(fileName)
                                 .arg(file.errorString()));
            return;
        }

        BusyTaskDialog* busy = new BusyTaskDialog("Loading file");
        busy->show();
        using namespace std::placeholders;

        PlotDataMap mapped_data = loader->readDataFromFile( &file,
                                                            [busy](int value) {
            busy->setValue(value);
            QApplication::processEvents();
        },
        [busy]() { return busy->wasCanceled(); } );

        busy->close();

        // remap to different type
        PlotDataMap::iterator it;

        size_t maxSizeX = 0;

        for ( it = mapped_data.begin(); it != mapped_data.end(); it++)
        {
            std::string name  = it->first;
            PlotDataPtr plot  = it->second;

            if( maxSizeX <  plot->getVectorX()->size() ){
                maxSizeX =  plot->getVectorX()->size();
            }

            QString qname = QString::fromStdString(name);
            // remap to derived class
            _mapped_plot_data.insert( std::make_pair( name, plot) );

            QColor color = colorHint();
            plot->setColorHint( color.red(), color.green(), color.blue() );

            curvelist_widget->list()->addItem( new QListWidgetItem( qname ) );
        }

        _undo_states.clear();
        _redo_states.clear();
        _undo_states.push_back(  xmlSaveState() );
        ui->horizontalSlider->setRange(0, maxSizeX );
    }
    else{
        qDebug() << "no loader found";
    }
}

void MainWindow::onActionReloadDataFile()
{
    onActionLoadDataFile( true );
}

void MainWindow::onActionLoadLayout()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open Layout",  QDir::currentPath(), "*.xml");
    if (fileName.isEmpty())
        return;

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

    xmlLoadState( domDocument );
    _undo_states.clear();
    _undo_states.push_back( domDocument );

}

/* TODO move to curve_selector.cpp

void MainWindow::on_pushButton_toggled(bool checked)
{
    for(int i=0;  i < settings_widgets.size(); i++)
    {
        QPropertyAnimation* m_anim = new QPropertyAnimation(settings_widgets[i], "maximumHeight");
        m_anim->setEasingCurve(checked ? QEasingCurve::OutQuad : QEasingCurve::OutExpo );

        m_anim->setStartValue( checked ? 0:25);
        m_anim->setEndValue( checked ? 25:0);
        m_anim->setDuration(checked ? 500 : 300);
        m_anim->setLoopCount(1); // forever
        m_anim->start(QAbstractAnimation::DeleteWhenStopped);
    }
}
*/

void MainWindow::on_pushHorizontalResize_pressed()
{
    currentPlotGrid()->maximizeHorizontalScale();
}

void MainWindow::on_pushVerticalResize_pressed()
{
    currentPlotGrid()->maximizeVerticalScale();
}

void MainWindow::on_pushLinkHorizontalScale_toggled(bool checked)
{
    _horizontal_link = checked;

    for (int index = 0; index < ui->tabWidget->count(); index++)
    {
        PlotMatrix* tab = currentPlotGrid();
        if (tab){
            tab->setHorizontalLink( _horizontal_link );
        }
    }
}

void MainWindow::on_pushButtonActivateTracker_toggled(bool checked)
{
    for (int index = 0; index < ui->tabWidget->count(); index++)
    {
        PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
        if (tab){
            tab->setActiveTracker( checked );
        }
    }
}

void MainWindow::on_pushButtonUndo_clicked(bool )
{
    if( _undo_states.size() > 1)
    {
        QDomDocument state_document = _undo_states.back();
        _redo_states.push_back( state_document );
        _undo_states.pop_back();
        state_document = _undo_states.back();
        xmlLoadState( state_document );
    }
}

void MainWindow::on_pushButtonRedo_clicked(bool )
{
    if( _redo_states.size() > 0)
    {
        QDomDocument state_document = _redo_states.back();
        _undo_states.push_back( state_document );
        _redo_states.pop_back();

        xmlLoadState( state_document );
    }
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
    tab->replot();
}

void MainWindow::on_tabWidget_tabCloseRequested(int index)
{
    PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );

    bool ask_confirmation = true;
    if( tab->widgetList().size() == 1 )
    {
        if( tab->widgetList().at(0)->isEmpty()){
            ask_confirmation = false;
        }
    }

    QMessageBox::StandardButton do_remove = QMessageBox::Yes;

    if( ask_confirmation )
    {
        ui->tabWidget->setCurrentIndex( index );
        QApplication::processEvents();

        do_remove = QMessageBox::question(0, tr("Warning"),
                                          tr("Do you really want to destroy this tab?\n") );
    }
    if( do_remove == QMessageBox::Yes )
    {
        // first add then delete.
        // Otherwise currentPlotGrid might be empty
        if( ui->tabWidget->count() == 1){
            on_addTabButton_pressed();
        }
        ui->tabWidget->removeTab( index );
    }
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    QSlider* slider = ui->horizontalSlider;
    double ratio = (double)position / (double)(slider->maximum() -  slider->minimum() );

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();

    for ( unsigned i = 0; i< currentPlotGrid()->widgetList().size(); i++ )
    {
        PlotWidget *plot =  currentPlotGrid()->widgetList().at(i);
        if( plot->isEmpty() == false)
        {
            QRectF bound_max = plot->maximumBoundingRect();
            if( minX > bound_max.left() )    minX = bound_max.left();
            if( maxX < bound_max.right() )   maxX = bound_max.right();
        }
    }
    double posX = (maxX-minX) * ratio;

    for ( unsigned i = 0; i< currentPlotGrid()->widgetList().size(); i++ )
    {
        PlotWidget *plot =  currentPlotGrid()->widgetList().at(i);
        plot->tracker()->manualMove( QPointF(posX,0) );
    }

    onTrackerTimeUpdated( posX );
}
