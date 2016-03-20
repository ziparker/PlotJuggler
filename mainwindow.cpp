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

QStringList  words_list;
int unique_number = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    words_list << "siam" << "tre" << "piccoli" << "porcellin"
               << "mai" << "nessun" << "ci" << "dividera";

    ui->listWidget->addItems( words_list );

    ui->splitter->setStretchFactor(0,0);
    ui->splitter->setStretchFactor(1,1);

    _horizontal_link = true;

    on_addTabButton_pressed();

    ui->pushHorizontalResize->setChecked( _horizontal_link );
    currentPlotGrid()->setHorizontalLink( _horizontal_link );

    connect(ui->actionSave_layout,SIGNAL(triggered()), this, SLOT(onActionSaveLayout()) );
    connect(ui->actionLoad_layout,SIGNAL(triggered()), this, SLOT(onActionLoadLayout()) );

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
    buildData();

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


void MainWindow::buildData()
{
    long size_plot = 100*1000;
    int index = 0;

    foreach( const QString& name, words_list)
    {
        _mapped_plot_data.insert( std::make_pair( name, PlotData(size_plot)));

        PlotData& plot_data = _mapped_plot_data.at(name);

        float A =  qrand()/(float)RAND_MAX * 6 - 3;
        float B =  qrand()/(float)RAND_MAX *3;
        float C =  qrand()/(float)RAND_MAX *3;
        float D =  qrand()/(float)RAND_MAX *2 -1;

        float t = 0;
        for (int indx=0; indx<size_plot; indx++)
        {
            plot_data.pushBack( QPointF( t, A*sin(B*t + C) +D*t*0.01 ) ) ;
            t += 0.001;
        }

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
        plot_data.setColorHint(color);
        index++;
    }

    ui->horizontalSlider->setRange(0, size_plot  );
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

void MainWindow::on_lineEdit_textChanged(const QString &arg1)
{
    Qt::CaseSensitivity cs = Qt::CaseInsensitive;
    if( ui->checkBoxCaseSensitive->isChecked())
    {
        cs = Qt::CaseSensitive;
    }
    QRegExp regexp( arg1,  cs, QRegExp::Wildcard );
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
            toHide =  name.contains(arg1, cs) == false;
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
    for( PlotDataMap::iterator it = _mapped_plot_data.begin(); it != _mapped_plot_data.end(); it++)
    {
        PlotData* plot = &(it->second);

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
    destination->addCurve( curve_name, &(_mapped_plot_data.at(curve_name)) );
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
        PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
        if (tab){
            tab->setHorizontalLink( _horizontal_link );
        }
    }
}
