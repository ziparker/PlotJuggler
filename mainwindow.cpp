#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMouseEvent>
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QStringListModel>
#include <QRegExpValidator>

#include <qwt_plot_canvas.h>

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

    on_addTabButton_pressed();

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
    QWidget * w =  this->childAt( event->pos() );
    if( w )
        qDebug()<< w->objectName();

    QMenu menu(this);
    menu.addAction(deleteOneAct);
    menu.addAction(deleteAllAct);
    menu.exec( event->globalPos() );
}


void MainWindow::mousePressEvent(QMouseEvent *)
{

}

void MainWindow::on_splitter_splitterMoved(int , int )
{
    QList<int> sizes = ui->splitter->sizes();
    int maxLeftWidth = ui->leftLayout->maximumSize().width();
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



void MainWindow::on_radioRegExp_toggled(bool )
{
    on_lineEdit_textChanged( ui->lineEdit->text() );
}

void MainWindow::on_checkBoxCaseSensitive_toggled(bool )
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
 /*   QGridLayout* grid =  ui->plotsLayout;
    ui->lcdNumber->display(value);

    for( PlotDataMap::iterator it = _mapped_plot_data.begin(); it != _mapped_plot_data.end(); it++)
    {
        PlotData* plot = &(it->second);

        float range = (float)ui->horizontalSlider->maximum() *0.001*0.1;
        plot->setRangeX( (float)value*0.001 , range ) ;
    }
    for (int index = 0; index < grid->count(); index++)
    {
        PlotWidget* plot = static_cast<PlotWidget*>( grid->itemAt(index)->widget() );
        if (plot){
            plot->updateAxes();
            plot->replot();
        }
    }*/
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

}
