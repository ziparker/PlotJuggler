#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMouseEvent>
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QStringListModel>
#include <QRegExpValidator>
#include <stdio.h>
#include <qwt_plot_canvas.h>
#include <QDomDocument>
#include <QFileDialog>
#include <QMessageBox>
#include <QPropertyAnimation>
#include <QStringRef>
#include <QThread>
#include "selectxaxisdialog.h"
#include "busydialog.h"
#include "busytaskdialog.h"

QStringList  words_list;
int unique_number = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    words_list << "siam" << "tre" << "piccoli" << "porcellin"
               << "mai" << "nessun" << "ci" << "dividera";


    ui->splitter->setStretchFactor(0,0);
    ui->splitter->setStretchFactor(1,1);

    _horizontal_link = true;

    on_addTabButton_pressed();

    ui->pushHorizontalResize->setChecked( _horizontal_link );
    currentPlotGrid()->setHorizontalLink( _horizontal_link );

    connect(ui->actionSave_layout,SIGNAL(triggered()), this, SLOT(onActionSaveLayout()) );
    connect(ui->actionLoad_layout,SIGNAL(triggered()), this, SLOT(onActionLoadLayout()) );
    connect(ui->actionLoadCSV,SIGNAL(triggered()), this, SLOT(onActionLoadCSV()) );

    for( int i=0; i< ui->gridLayoutSettings->count(); i++)
    {
        QLayoutItem* item = ui->gridLayoutSettings->itemAt(i);
        if(item)
        {
            QWidget* widget = item->widget();
            if(widget) {
                widget->setMaximumHeight( 0 );
                settings_widgets.push_back( widget);
            }
        }
    }

    createActions();

    ui->menuBar->hide();
    ui->mainToolBar->setMovable(false);
    ui->mainToolBar->addAction( ui->menuFile->menuAction());

}

MainWindow::~MainWindow()
{
    delete ui;
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
    /* deleteAllAct = new QAction(tr("&Delete All"), this);
    deleteAllAct->setStatusTip(tr("Delete the plot"));
    connect(deleteAllAct, SIGNAL(triggered()), this, SLOT(deletePlot()));

    deleteOneAct = new QAction(tr("&Delete One"), this);
    deleteOneAct->setStatusTip(tr("Delete the plot"));
    connect(deleteOneAct, SIGNAL(triggered()), this, SLOT(deletePlot()));*/
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

void MainWindow::buildData()
{
    long SIZE = 100*1000;

    ui->listWidget->addItems( words_list );
    QSharedPointer<std::vector<double> > t_vector ( new std::vector<double>());
    t_vector->reserve(SIZE);

    double t = 0;
    for (int indx=0; indx<SIZE; indx++)
    {
        t_vector->push_back( t );
        t += 0.001;
    }

    foreach( const QString& name, words_list)
    {
        QSharedPointer<std::vector<double> > y_vector( new std::vector<double>() );
        y_vector->reserve(SIZE);

        _mapped_raw_data.insert( std::make_pair( name, y_vector));

        float A =  qrand()/(float)RAND_MAX * 6 - 3;
        float B =  qrand()/(float)RAND_MAX *3;
        float C =  qrand()/(float)RAND_MAX *3;
        float D =  qrand()/(float)RAND_MAX *2 -1;

        for (int indx=0; indx<SIZE; indx++)
        {
            double x = t_vector->at(indx);
            y_vector->push_back(  A*sin(B*x + C) +D*x*0.02 ) ;
        }

        PlotData* plot = new PlotData(t_vector, y_vector);
        plot->setColorHint( colorHint() );
        _mapped_plot_data.insert( std::make_pair( name, plot));

    }

    ui->horizontalSlider->setRange(0, SIZE  );
    on_horizontalSlider_valueChanged(0);

}



void MainWindow::contextMenuEvent(QContextMenuEvent *event)
{
    /*  QWidget * w =  this->childAt( event->pos() );
    if( w )
        qDebug()<< w->objectName();

    QMenu menu(this);
    menu.addAction(deleteOneAct);
    menu.addAction(deleteAllAct);
    menu.exec( event->globalPos() );*/
}


void MainWindow::mousePressEvent(QMouseEvent *)
{

}

void MainWindow::on_splitter_splitterMoved(int , int )
{
    QList<int> sizes = ui->splitter->sizes();
    int maxLeftWidth = ui->listWidget->maximumWidth();
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

void MainWindow::on_lineEdit_textChanged(const QString &search_string)
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



void MainWindow::on_radioRegExp_toggled(bool checked)
{
    if(checked)
    {
        ui->radioContains->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void MainWindow::on_checkBoxCaseSensitive_toggled(bool checked)
{

    on_lineEdit_textChanged( ui->lineEdit->text() );
}



void MainWindow::on_pushAddRow_pressed()
{
    currentPlotGrid()->addRow();
}



void MainWindow::on_pushAddColumn_pressed()
{
    currentPlotGrid()->addColumn();
}


void MainWindow::on_pushremoveEmpty_pressed()
{
    PlotMatrix *grid = currentPlotGrid();
    qDebug() << "-------------";

    for( int row = 0; row< grid->numRows(); row++)
    {
        while( grid->isRowEmpty( row ) && row < grid->numRows() ){
            grid->removeRow( row );
        }
    }

    for( int col = 0; col< grid->numColumns(); col++)
    {
        while( grid->isColumnEmpty( col ) && col < grid->numColumns() ){
            grid->removeColumn( col );
        }
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    std::map<QString, PlotData*>::iterator it;
    for( it = _mapped_plot_data.begin(); it != _mapped_plot_data.end(); it++)
    {
        PlotData* plot = (it->second);

        float range = (float)ui->horizontalSlider->maximum() *0.001*0.1;
        //  plot->setRangeX( (float)value*0.001 , range ) ;
    }

    for (int index = 0; index < ui->tabWidget->count(); index++)
    {
        PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
        if (tab){
            tab->updateLayout();
        }
    }
}

void MainWindow::on_plotAdded(PlotWidget *widget)
{
    connect(widget,SIGNAL(curveNameDropped(QString, PlotWidget*)),
            this,  SLOT(addCurveToPlot(QString, PlotWidget*)));
}

void MainWindow::addCurveToPlot(QString curve_name, PlotWidget* destination)
{
    destination->addCurve( curve_name, (_mapped_plot_data.at(curve_name)) );
}

PlotMatrix *MainWindow::currentPlotGrid()
{
    return static_cast<PlotMatrix*>( ui->tabWidget->currentWidget() );
}

void MainWindow::on_addTabButton_pressed()
{
    PlotMatrix* grid = new PlotMatrix(this);
    ui->tabWidget->addTab( grid, QString("plot") );
    connect( grid, SIGNAL(plotAdded(PlotWidget*)), this, SLOT(on_plotAdded(PlotWidget*)));

    ui->tabWidget->setCurrentWidget( grid );
    grid->setHorizontalLink( _horizontal_link );
}

void MainWindow::onActionSaveLayout()
{
    QDomDocument doc;
    QDomProcessingInstruction instr = doc.createProcessingInstruction(
                "xml", "version='1.0' encoding='UTF-8'");

    doc.appendChild(instr);

    QDomElement root = doc.createElement( "root" );

    qDebug() << ">> add root";
    for(int i=0; i< ui->tabWidget->count(); i++)
    {
        PlotMatrix* widget = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );
        QDomElement element = widget->getDomElement(doc);

        root.appendChild( element );
    }

    doc.appendChild(root);

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

void MainWindow::onActionLoadCSV()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open Layout",  QDir::currentPath(), "*.csv");
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

    int linecount = 0;

    {
        QTextStream inA(&file);

        BusyDialog* progressDialog = new BusyDialog(this);

        QApplication::processEvents();

        while (!inA.atEnd())
        {
            inA.readLine();
            linecount++;
            if(linecount%1000 == 0) {

                qDebug() << linecount;
                QApplication::processEvents();
            }
        }
        file.close();
        progressDialog->close();
    }


    file.open(QFile::ReadOnly);
    QTextStream inB(&file);

    std::vector<SharedVector> ordered_vectors;
    std::vector<QString> ordered_names;

    bool first_line = true;

    BusyTaskDialog* taskDialog = 0;

    int time_index = -1;
    int tot_lines = linecount -1;
    linecount = 0;
    double prev_time = -1;
    double time_offset = 0;
    bool use_time_bias = false;

    while (!inB.atEnd())
    {
        QString line = inB.readLine();

        QStringList string_items = line.split(',');

        if( first_line )
        {
            for (int i=0; i < string_items.size(); i++ )
            {
                QStringRef field_name ( &string_items[i] );
                if( field_name.startsWith( "field." ) )
                {
                    field_name = field_name.mid(6);
                }

                QString name = field_name.toString();

                SharedVector data_vector( new std::vector<double>());
                data_vector->reserve(tot_lines);

                PlotData* plot = new PlotData;
                plot->setColorHint( colorHint() );
                plot->setName( name );

                ordered_vectors.push_back( data_vector );
                ordered_names.push_back( name );

                _mapped_raw_data.insert( std::make_pair( name, data_vector ) );
                _mapped_plot_data.insert( std::make_pair( name, plot ) );
            }

            QDialog* dialog = new selectXAxisDialog( &string_items, this);
            dialog->exec();
            time_index = dialog->result();

            first_line = false;
            if( ordered_names[time_index].compare("%time") == 0)
            {
                use_time_bias = true;
            }
        }
        else{
            if( !taskDialog )
            {
                taskDialog = new BusyTaskDialog("Loading file");
                taskDialog->show();
            }

            if( use_time_bias && time_offset <= 0){
                time_offset = string_items[ time_index ].toDouble();
            }

            double t = string_items[ time_index ].toDouble();

            if( use_time_bias )
            {
                t -= time_offset;
                t *= 0.000000001;
            }

            if( t <= prev_time)
            {
                QMessageBox::warning(this, tr("Error reading file"),
                                     tr("Selected time in not monotonic") );
                taskDialog->cancel();
                break;
            }
            prev_time = t;

            for (int i=0; i < string_items.size(); i++ )
            {
                double y = string_items[i].toDouble();
                ordered_vectors[i]->push_back( y );
            }
            if(linecount++ %100 == 0)
            {
                taskDialog->setValue( (100* linecount) / tot_lines );
                QApplication::processEvents();
                if( taskDialog->wasCanceled() )
                {
                    break;
                }
            }
        }
    }
    file.close();

    if( taskDialog->wasCanceled() == false)
    {
        SharedVector time_vector = ordered_vectors[time_index];

        for( int i=0; i < ordered_vectors.size(); i++)
        {
            QString name = ordered_names[i];
            _mapped_plot_data[ name ]->addData( time_vector, ordered_vectors[i]);

            ui->listWidget->addItem( new QListWidgetItem( name ) );
        }
    }
    else{
        while( _mapped_plot_data.size() > 0)
        {
            delete _mapped_plot_data.begin()->second;
            _mapped_plot_data.erase( _mapped_plot_data.begin() );
        }
    }

    taskDialog->close();

    qDebug() << "DONE";

}

void MainWindow::onActionLoadLayout()
{
    while(ui->tabWidget->count()>0)
    {
        ui->tabWidget->removeTab(0);
    }
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
    int errorLine;
    int errorColumn;

    QDomDocument domDocument;

    if (!domDocument.setContent(&file, true, &errorStr, &errorLine, &errorColumn)) {
        QMessageBox::information(window(), tr("XML Layout"),
                                 tr("Parse error at line %1:\n%2")
                                 .arg(errorLine)
                                 .arg(errorStr));
        return;
    }

    QDomElement root = domDocument.namedItem("root").toElement();
    if ( root.isNull() ) {
        qWarning() << "No <root> element found at the top-level of the XML file!";
        return ;
    }

    QDomElement plotmatrix;
    for (  plotmatrix = root.firstChildElement( "plotmatrix" )  ;
           !plotmatrix.isNull();
           plotmatrix = plotmatrix.nextSiblingElement( "plotmatrix" ) )
    {
        if( !plotmatrix.hasAttribute("rows") || !plotmatrix.hasAttribute("columns") )
        {
            qWarning() << "No [rows] or [columns] attribute in <plotmatrix> XML file!";
            return ;
        }
        int rows = plotmatrix.attribute("rows").toInt();
        int columns = plotmatrix.attribute("columns").toInt();

        // add the tab
        this->on_addTabButton_pressed();
        for(int c = 0; c<columns; ++c)
        {
            currentPlotGrid()->addColumn();
        }
        for(int r = 1; r<rows; ++r)
        {
            currentPlotGrid()->addRow();
        }
        //----------------
        QDomElement plot;
        for (  plot = plotmatrix.firstChildElement( "plot" )  ;
               !plot.isNull();
               plot = plot.nextSiblingElement( "plot" ) )
        {
            if( !plot.hasAttribute("row") || !plot.hasAttribute("col") )
            {
                qWarning() << "No [row] or [col] attribute in <plot> XML file!";
                return ;
            }
            int row = plot.attribute("row").toInt();
            int col = plot.attribute("col").toInt();

            PlotWidget *plot_widget = currentPlotGrid()->plotAt(row,col);
            //-----------------------------
            QDomElement curve;
            for (  curve = plot.firstChildElement( "curve" )  ;
                   !curve.isNull();
                   curve = curve.nextSiblingElement( "curve" ) )
            {
                if( !curve.hasAttribute("name") )
                {
                    qWarning() << "No [name] attribute in <plot> XML file!";
                    return ;
                }
                QString curve_name = curve.attribute("name");
                this->addCurveToPlot( curve_name, plot_widget);
            }
        }
    }
}


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

void MainWindow::on_radioFlatView_toggled(bool checked)
{
    if(checked)
    {
        ui->radioTreeView->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void MainWindow::on_radioTreeView_toggled(bool checked)
{
    if(checked)
    {
        ui->radioFlatView->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

void MainWindow::on_radioContains_toggled(bool checked)
{
    if(checked)
    {
        ui->radioRegExp->setChecked( false);
        on_lineEdit_textChanged( ui->lineEdit->text() );
    }
}

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
