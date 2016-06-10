#include <QMenu>
#include <QAction>
#include <QInputDialog>
#include <QMouseEvent>
#include "tabbedplotwidget.h"
#include "ui_tabbedplotwidget.h"


TabbedPlotWidget::TabbedPlotWidget(PlotDataMap *mapped_data,  MainWindow* main_window, QWidget *parent ) :
    QWidget(parent),
    ui(new Ui::TabbedPlotWidget)
{
    _main_window = (QWidget*)main_window;
    _mapped_data = mapped_data;
    _parent_type = QString("floating_window");

    ui->setupUi(this);

    init();
}

TabbedPlotWidget::TabbedPlotWidget(PlotDataMap *mapped_data, QWidget* main_window_parent ) :
    QWidget( main_window_parent ),
    ui(new Ui::TabbedPlotWidget)
{
    _main_window = main_window_parent;
    _mapped_data = mapped_data;
    _parent_type = QString("main_window");
    ui->setupUi(this);

    init();
}

void TabbedPlotWidget::init()
{
    _horizontal_link = true;
    addTab();

    ui->tabWidget->tabBar()->installEventFilter( this );

    _action_renameTab = new QAction(tr("rename tab"), this);
    _action_moveTab   = new QAction(tr("move tab"), this);

    connect( _action_renameTab, SIGNAL(triggered()), this, SLOT(changeCurrentTabName()) );
    connect( _action_moveTab, SIGNAL(triggered()), _main_window, SLOT(on_createFloatingWindow()) );

    _tab_menu = new QMenu(this);
    _tab_menu->addAction( _action_renameTab );
    _tab_menu->addAction( _action_moveTab );
}

PlotMatrix *TabbedPlotWidget::currentTab()
{
    return static_cast<PlotMatrix*>( ui->tabWidget->currentWidget() );
}

QTabWidget *TabbedPlotWidget::tabWidget()
{
    return ui->tabWidget;
}

void TabbedPlotWidget::addTab()
{
    PlotMatrix* tab = new PlotMatrix( _mapped_data, this);

    ui->tabWidget->addTab( tab, QString("plot") );

    connect( tab, SIGNAL(plotAdded(PlotWidget*)), _main_window, SLOT(on_plotAdded(PlotWidget*)));
    connect( tab, SIGNAL(layoutModified()),       _main_window, SLOT( on_undoableChange()) );

    ui->tabWidget->setCurrentWidget( tab );

    tab->setHorizontalLink( _horizontal_link );

    //TODO  grid->setActiveTracker( ui->pushButtonActivateTracker->isChecked() );

    on_pushAddColumn_pressed();
}

QDomElement TabbedPlotWidget::xmlSaveState(QDomDocument &doc)
{
    QDomElement tabbed_area = doc.createElement( "tabbed_widget" );

    tabbed_area.setAttribute("parent", _parent_type);

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

    currentTab()->replot();
    return true;
}


TabbedPlotWidget::~TabbedPlotWidget()
{
    delete ui;
}

void TabbedPlotWidget::changeCurrentTabName()
{
    int idx = ui->tabWidget->tabBar()->currentIndex ();

    bool ok = true;
    QString newName = QInputDialog::getText (
                this, tr ("Change Name of the selected tab"),
                tr ("Insert New Tab Name"),
                QLineEdit::Normal,
                ui->tabWidget->tabText (idx),
                &ok);

    if (ok) {
        ui->tabWidget->setTabText (idx, newName);
    }
}

void TabbedPlotWidget::on_pushAddColumn_pressed()
{
    currentTab()->addColumn();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushVerticalResize_pressed()
{
    currentTab()->maximizeHorizontalScale();
}

void TabbedPlotWidget::on_pushHorizontalResize_pressed()
{
    currentTab()->maximizeVerticalScale();
}

void TabbedPlotWidget::on_pushAddRow_pressed()
{
    currentTab()->addRow();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_addTabButton_pressed()
{
    addTab();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushremoveEmpty_pressed()
{
    PlotMatrix *tab = currentTab();

    for( int row = 0; row< tab->numRows(); row++)
    {
        while( tab->isRowEmpty( row ) && row < tab->numRows() ){
            tab->removeRow( row );
        }
    }

    for( int col = 0; col< tab->numColumns(); col++)
    {
        while( tab->isColumnEmpty( col ) && col < tab->numColumns() ){
            tab->removeColumn( col );
        }
    }

    if( tab->numColumns() == 0 &&  tab->numRows() == 0 )
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
        emit undoableChangeHappened();
    }
}

void TabbedPlotWidget::on_buttonLinkHorizontalScale_toggled(bool checked)
{
    _horizontal_link = checked;

    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );
        tab->setHorizontalLink( _horizontal_link );
    }
}


bool TabbedPlotWidget::eventFilter(QObject *obj, QEvent *event)
{
    QTabBar* tab_bar = ui->tabWidget->tabBar();

    if (obj == tab_bar )
    {
        if( event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent *mouse_event = (QMouseEvent *)event;

            int index = tab_bar->tabAt( mouse_event->pos() );
            tab_bar->setCurrentIndex( index );

            if( mouse_event->button() == Qt::RightButton )
            {
                _tab_menu->exec( mouse_event->globalPos() );
            }
        }
    }

    // Standard event processing
    return QObject::eventFilter(obj, event);
}
