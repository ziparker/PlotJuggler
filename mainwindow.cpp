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

    addPlotWidget(QString("0 _ 0"), 0, 0);

    _num_active_columns = 1;
    _num_active_rows = 1;

    /*ui->plotsLayout->setColumnStretch(0,0);
    ui->plotsLayout->setRowStretch(0,0);

    ui->plotsLayout->setColumnStretch(1,0);
    ui->plotsLayout->setColumnStretch(2,0);
    ui->plotsLayout->setColumnStretch(3,0);

    ui->plotsLayout->setRowStretch(1,0);
    ui->plotsLayout->setRowStretch(2,0);
    ui->plotsLayout->setRowStretch(3,0);*/

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

    QwtPlotCanvas *canvas = new QwtPlotCanvas(plot_widget);
    canvas->setFrameStyle( QFrame::NoFrame );
    canvas->setPaintAttribute( QwtPlotCanvas::BackingStore, false );

    plot_widget->setCanvas( canvas );
    plot_widget->setCanvasBackground( QColor( 250, 250, 250 ) );

    ui->plotsLayout->addWidget(plot_widget,row,col);

    connect( plot_widget, SIGNAL(swapWidgets(QString,QString)), this, SLOT(swapWidgets(QString,QString)) );

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
            plot_data.pushBack( QPointF( t, A*sin(B*t + C) +D ) ) ;
            t += 0.00025;
        }
    }
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

void MainWindow::rearrangeGridLayout()
{
   /* QGridLayout* grid= ui->plotsLayout;
    int cols = grid->columnCount();
    int rows = grid->rowCount();

    for(int c=0; c<cols; c++ )
    {
        if( isColumnEmpty(grid, c) )
        {
            for(int cc = c+1; cc< cols; cc++)
            {
                for(int r=0; r< rows; r++)
                {
                    QLayoutItem *item = grid->itemAtPosition(r, cc);
                    if( item ){
                        grid->removeItem( item );
                        grid->addWidget(item->widget(), r, cc-1);
                    }
                }
            }
        }
    }

    for(int r=0; r<rows; r++ )
    {
        if( isRowEmpty(grid, r) )
        {
            for(int rr = r+1; rr< rows; rr++)
            {
                for(int c=0; c< cols; c++)
                {
                    QLayoutItem *item = grid->itemAtPosition(rr, c);
                    if( item ){
                        grid->removeItem( item );
                        grid->addWidget(item->widget(), rr-1, c);
                    }
                }
            }
        }
    }*/
}

void MainWindow::resizePlots(QSize layout_size)
{
    QSize widget_size;
    widget_size.setWidth( 100 );
    widget_size.setHeight( 100 );

    for (int i=0; i<ui->plotsLayout->count(); i++)
    {
        QLayoutItem* item = ui->plotsLayout->itemAt(i);
        if( item )
        {
            item->widget()->resize( widget_size );
            qDebug() << "resize to " << widget_size;
        }
    }
}


void MainWindow::on_pushAddRow_pressed()
{
   // QSize layout_size = ui->plotsLayout->sizeHint();
 //   rearrangeGridLayout();
    int cols = ui->plotsLayout->columnCount();
    int rows = ui->plotsLayout->rowCount();

    qDebug() << rows << " " << cols;

    if( _num_active_rows < 4){
        for (int c=0; c< _num_active_columns; c++)
        {
            QString name = QString::number( _num_active_rows ) + QString(" / ") + QString::number( c );
            addPlotWidget(name, _num_active_rows, c);
        }
        _num_active_rows++;
    }
   // resizePlots(layout_size);
}



void MainWindow::on_pushAddColumn_pressed()
{
  //  rearrangeGridLayout();
    int cols = ui->plotsLayout->columnCount();
    int rows = ui->plotsLayout->rowCount();

    qDebug() << rows << " " << cols;

    if( _num_active_columns < 4){
        for (int r=0; r< _num_active_rows; r++)
        {
            QString name = QString::number( r ) + QString(" / ") + QString::number( _num_active_columns );
            addPlotWidget( name, r, _num_active_columns );
        }
        _num_active_columns++;
    }
}


void MainWindow::on_pushAddPlot_pressed()
{
    ui->plotsLayout->setSpacing(0);
    int width = 0;
    int height = 0;

    for (int c=0; c< _num_active_columns; c++) {
        width += ui->plotsLayout->itemAtPosition(0,c)->widget()->size().width();
        qDebug() << "width " <<width;
    }

    for (int r=0; r< _num_active_rows; r++){
        height += ui->plotsLayout->itemAtPosition(r,0)->widget()->size().height();
        qDebug() << "height " << height;
    }

    QSize new_size (width / _num_active_columns, height / _num_active_rows);

    for (int r=0; r< _num_active_rows; r++){
        for (int c=0; c< _num_active_columns; c++){

            ui->plotsLayout->itemAtPosition(r,c)->widget()->resize( new_size );
            qDebug() << ui->plotsLayout->itemAtPosition(r,c)->widget()->size();
        }
    }
}
