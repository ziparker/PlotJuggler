#include "tabbedplotwidget.h"
#include "ui_tabbedplotwidget.h"


TabbedPlotWidget::TabbedPlotWidget(PlotDataMap *mapped_data,  MainWindow* main_window, QWidget *parent ) :
    QWidget(parent),
    ui(new Ui::TabbedPlotWidget)
{
    _main_window = main_window;
    _mapped_data = mapped_data;
    ui->setupUi(this);
    addTab();
}

PlotMatrix *TabbedPlotWidget::currentPlotGrid()
{
    return static_cast<PlotMatrix*>( ui->tabWidget->currentWidget() );
}

QTabWidget *TabbedPlotWidget::tabWidget()
{
    return ui->tabWidget;
}

void TabbedPlotWidget::addTab()
{
    PlotMatrix* grid = new PlotMatrix( _mapped_data, this);

    ui->tabWidget->addTab( grid, QString("plot") );

    connect( grid, SIGNAL(plotAdded(PlotWidget*)), (QWidget *)_main_window, SLOT(on_plotAdded(PlotWidget*)));
    connect( grid, SIGNAL(layoutModified()),       (QWidget *)_main_window, SLOT( on_undoableChange()) );

    ui->tabWidget->setCurrentWidget( grid );
   //TODO   grid->setHorizontalLink( _horizontal_link );

   //TODO  grid->setActiveTracker( ui->pushButtonActivateTracker->isChecked() );

    on_pushAddColumn_pressed();

    emit undoableChangeHappened();
}

QDomElement TabbedPlotWidget::xmlSaveState(QDomDocument &doc)
{
    QDomElement tabbed_area = doc.createElement( "tabbed_widget" );

    if( (QWidget*)_main_window == this->parent())
    {
        tabbed_area.setAttribute("parent", "mainwindow");
    }
    else{
        tabbed_area.setAttribute("parent", "dialog");
    }


    for(int i=0; i< ui->tabWidget->count(); i++)
    {
        PlotMatrix* widget = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );
        QDomElement element = widget->xmlSaveState(doc);

        element.setAttribute("tab_name",  ui->tabWidget->tabText(i) );
        tabbed_area.appendChild( element );
    }

    QDomElement current_plotmatrix =  doc.createElement( "currentPlotMatrix" );
    current_plotmatrix.setAttribute( "index", ui->tabWidget->currentIndex() );
    tabbed_area.appendChild( current_plotmatrix );

    return tabbed_area;
}

bool TabbedPlotWidget::xmlLoadState(QDomElement &tabbed_area)
{
    int num_tabs =  ui->tabWidget->count();
    int index = 0;

    QDomElement plotmatrix_el;

    for (  plotmatrix_el = tabbed_area.firstChildElement( "plotmatrix" )  ;
           !plotmatrix_el.isNull();
           plotmatrix_el = plotmatrix_el.nextSiblingElement( "plotmatrix" ) )
    {
        // add if tabs are too few
        if( index == num_tabs)
        {
            this->addTab();
            num_tabs++;
        }
        PlotMatrix* plot_matrix = static_cast<PlotMatrix*>(  ui->tabWidget->widget(index) );
        bool success = plot_matrix->xmlLoadState( plotmatrix_el );

        // read tab name
        if( plotmatrix_el.hasAttribute("tab_name"))
        {
             ui->tabWidget->setTabText( index, plotmatrix_el.attribute("tab_name" ) );
        }

        if( !success )
        {
            return false;
        }

        index++;
    }

    // remove if tabs are too much
    while( num_tabs > index ){
         ui->tabWidget->removeTab( num_tabs-1 );
        num_tabs--;
    }

    QDomElement current_plotmatrix =  tabbed_area.firstChildElement( "currentPlotMatrix" );
    int current_index = current_plotmatrix.attribute( "index" ).toInt();
    ui->tabWidget->setCurrentIndex( current_index );

    currentPlotGrid()->replot();
    return true;
}

TabbedPlotWidget::~TabbedPlotWidget()
{
    delete ui;
}

void TabbedPlotWidget::on_pushAddColumn_pressed()
{
    currentPlotGrid()->addColumn();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushVerticalResize_pressed()
{
    currentPlotGrid()->maximizeHorizontalScale();
}

void TabbedPlotWidget::on_pushHorizontalResize_pressed()
{
    currentPlotGrid()->maximizeVerticalScale();
}

void TabbedPlotWidget::on_pushAddRow_pressed()
{
    currentPlotGrid()->addRow();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_addTabButton_pressed()
{
    addTab();
}

void TabbedPlotWidget::on_pushremoveEmpty_pressed()
{
    PlotMatrix *grid = currentPlotGrid();

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

    if( grid->numColumns() == 0 &&  grid->numRows() == 0 )
    {
        on_pushAddColumn_pressed();
    }
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_tabWidget_currentChanged(int index)
{
    PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
    tab->replot();
}

void TabbedPlotWidget::on_tabWidget_tabCloseRequested(int index)
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
                                          tr("Do you really want to destroy this tab?\n"),
                                          QMessageBox::Yes | QMessageBox::No,
                                          QMessageBox::No );
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
