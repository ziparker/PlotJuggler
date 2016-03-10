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

    createActions();
    buildData();

    _num_active_columns = 0;
    _num_active_rows = 0;
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

void MainWindow::dragMoveEvent(QDragMoveEvent *event)
{

}

void MainWindow::dropEvent(QDropEvent *event)
{

}


void MainWindow::addPlotWidget(QString name,int row,int col)
{
    qDebug() <<" addign plot to " << row << " " << col;

    PlotWidget* plot_widget = new PlotWidget( &_mapped_plot_data, this );
    plot_widget->setWindowTitle( name );

    _plotWidgets.push_back( plot_widget );

    // plot_widget->setTitle( QString("Plot") + QString::number(_plotWidgets.size())  );

    ui->plotsLayout->addWidget(plot_widget,row,col);

    connect( plot_widget, SIGNAL(swapWidgets(QString,QString)), this, SLOT(swapWidgets(QString,QString)) );

    plot_widget->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::createActions()
{
    deleteAllAct = new QAction(tr("&Delete All"), this);
    deleteAllAct->setStatusTip(tr("Delete the plot"));
    connect(deleteAllAct, SIGNAL(triggered()), this, SLOT(deletePlot()));

    deleteOneAct = new QAction(tr("&Delete One"), this);
    deleteOneAct->setStatusTip(tr("Delete the plot"));
    connect(deleteOneAct, SIGNAL(triggered()), this, SLOT(deletePlot()));
}

void MainWindow::deletePlot()
{

}

void MainWindow::buildData()
{
    long size_plot = 100*1000;

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



void MainWindow::swapWidgets(QString src, QString dst)
{
    QWidget* widgetA = 0;
    QWidget* widgetB = 0;

    int x1,y1, x2,y2, span;

    QGridLayout* grid = ui->plotsLayout;

    for(int i=0; i< grid->count(); i++)
    {
        QLayoutItem * item = grid->itemAt(i);

        if(dynamic_cast<QWidgetItem *>(item))   //    <-- Note! QWidgetItem, and not QWidget!
        {
            if( QString::compare(item->widget()->windowTitle(), src ) == 0){
                widgetA = item->widget();
                grid->getItemPosition(i, &x1, &y1, &span, &span);
            }
            else if (QString::compare(item->widget()->windowTitle(), dst ) == 0){
                widgetB = item->widget();
                grid->getItemPosition(i, &x2, &y2, &span, &span);
            }
        }
    }
    if( widgetA && widgetB)
    {
        grid->removeWidget(widgetA);
        grid->removeWidget(widgetB);
        grid->addWidget(widgetA, x2, y2, 1, 1);
        grid->addWidget(widgetB, x1, y1, 1, 1);
    }
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

bool isColumnEmpty(QGridLayout* grid, int col )
{
    for (int r=0; r< grid->rowCount(); r++)
    {
        QLayoutItem *item = grid->itemAtPosition(r, col);
        if( item  ){
            PlotWidget *widget = static_cast<PlotWidget *>( item->widget() );
            if (widget && widget->isEmpty() == false)
            {
                return false;
            }
        }
    }
    return true;
}

bool isRowEmpty(QGridLayout* grid, int row )
{
    for (int c=0; c< grid->columnCount(); c++)
    {
        QLayoutItem *item = grid->itemAtPosition(row, c);
        if( item  ){
            PlotWidget *widget = static_cast<PlotWidget *>( item->widget() );
            if (widget && widget->isEmpty() == false)
            {
                return false;
            }
        }
    }
    return true;
}


void MainWindow::on_pushAddRow_pressed()
{
    int cols = ui->plotsLayout->columnCount();
    int rows = ui->plotsLayout->rowCount();

    qDebug() << rows << " " << cols;

    if( !_num_active_columns || !_num_active_columns) {
        addPlotWidget(QString("0 _ 0"), 0, 0);
        _num_active_columns = 1;
        _num_active_rows = 1;
        return;
    }

    if( _num_active_rows < 4){
        for (int c=0; c< _num_active_columns; c++)
        {
            QString name = QString::number( unique_number++  );
            addPlotWidget(name, _num_active_rows, c);
        }
        _num_active_rows++;
    }
}



void MainWindow::on_pushAddColumn_pressed()
{
    //  rearrangeGridLayout();
    int cols = ui->plotsLayout->columnCount();
    int rows = ui->plotsLayout->rowCount();

    qDebug() << rows << " " << cols;

    if( !_num_active_columns || !_num_active_columns)  {
        addPlotWidget(QString("0 _ 0"), 0, 0);
        _num_active_columns = 1;
        _num_active_rows = 1;
        return;
    }

    if( _num_active_columns < 4){
        for (int r=0; r< _num_active_rows; r++)
        {
            QString name = QString::number( unique_number++ ) ;
            addPlotWidget( name, r, _num_active_columns );
        }
        _num_active_columns++;
    }
}


void MainWindow::on_pushremoveEmpty_pressed()
{
    QGridLayout* grid =  ui->plotsLayout;
    for(int c=0; c< _num_active_columns ; c++ )
    {
        if( isColumnEmpty(grid, c) && c < _num_active_columns )
        {
            // move the right column
            for(int cc = c+1; cc< _num_active_columns; cc++)
            {
                if (  isColumnEmpty(grid, cc) == false)
                {
                    for(int r=0; r< _num_active_rows; r++)
                    {
                        QWidget *widgetA = grid->itemAtPosition(r, cc)->widget();
                        QWidget *widgetB = grid->itemAtPosition(r, c )->widget();
                        grid->removeWidget(widgetA);
                        grid->removeWidget(widgetB);
                        grid->addWidget(widgetA, r, c);
                        grid->addWidget(widgetB, r, cc);
                    }
                }
            }
        }
    }
    //-------------------------------
    for(int r=0; r< _num_active_rows ; r++ )
    {
        if( isRowEmpty(grid, r) && r < _num_active_rows )
        {
            // move the right column
            for(int rr = r+1; rr< _num_active_rows; rr++)
            {
                if (  isRowEmpty(grid, rr) == false)
                {
                    for(int c=0; c<_num_active_columns; c++)
                    {
                        QWidget *widgetA = grid->itemAtPosition(rr, c)->widget();
                        QWidget *widgetB = grid->itemAtPosition(r,  c )->widget();
                        grid->removeWidget(widgetA);
                        grid->removeWidget(widgetB);
                        grid->addWidget(widgetA, r, c);
                        grid->addWidget(widgetB, rr, c);
                    }
                }
            }
        }
    }

    //--------------------------------
    while( isColumnEmpty(grid, _num_active_columns-1) && _num_active_columns >0 )
    {
        for(int r=0; r< _num_active_rows; r++) {
            grid->itemAtPosition(r, _num_active_columns-1)->widget()->close();
        }
        _num_active_columns--;
    }

    while( isRowEmpty(grid, _num_active_rows-1) && _num_active_rows >0 )
    {
        for(int c=0; c< _num_active_columns; c++) {
            grid->itemAtPosition( _num_active_rows-1, c )->widget()->close();
        }
        _num_active_rows--;
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    QGridLayout* grid =  ui->plotsLayout;
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
    }
}
