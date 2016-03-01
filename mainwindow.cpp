#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMouseEvent>
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QStringListModel>
#include <QRegExpValidator>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    for (int i=0; i<4; i++)
    {
        DragableWidget* widget = new DragableWidget( this );
        widget->setWindowTitle( QString::number(i) );
        widget->setReadOnly( true );
        editors.push_back( widget );
        widget->append( QString::number(i));
        ui->plotsLayout->addWidget(widget,i%2,i/2,1,1);

        connect( widget, SIGNAL(swapWidgets(QString,QString)), this, SLOT(swapWidgets(QString,QString)) );
    }

    QStringList  list;

    list << "siam" << "tre" << "piccoli" << "porcellin"
         << "mai" << "nessun" << "ci" << "dividera";

    ui->listWidget->addItems( list);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_2_pressed()
{

}

void MainWindow::on_pushButton_pressed()
{

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

    // ui->plotsLayout->replaceWidget()
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
    QRegExp regexp( arg1,  cs );
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

void MainWindow::on_checkBoxCaseSensitive_toggled(bool checked)
{
    on_lineEdit_textChanged( ui->lineEdit->text() );
}
