#include <QMenu>
#include <QSignalMapper>
#include <QAction>
#include <QInputDialog>
#include <QMouseEvent>
#include "tabbedplotwidget.h"
#include "ui_tabbedplotwidget.h"


TabbedPlotWidget::TabbedPlotWidget(PlotDataMap *mapped_data, PlotMatrix *first_tab,  MainWindow* main_window, QWidget *parent ) :
    QWidget(parent),
    ui(new Ui::TabbedPlotWidget)
{
    _main_window = (QWidget*)main_window;
    _mapped_data = mapped_data;
    _parent_type = QString("floating_window");

    ui->setupUi(this);

    _horizontal_link = true;
    addTab( first_tab );

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

    _horizontal_link = true;
    addTab( NULL );

    init();
}

void TabbedPlotWidget::init()
{
    ui->tabWidget->tabBar()->installEventFilter( this );

    _action_renameTab = new QAction(tr("Rename tab"), this);

    connect( _action_renameTab, SIGNAL(triggered()), this, SLOT(renameCurrentTab()) );
    connect( this, SIGNAL(sendTabToNewWindow(PlotMatrix*)), _main_window, SLOT(on_createFloatingWindow(PlotMatrix*)) );

    _tab_menu = new QMenu(this);
    _tab_menu->addAction( _action_renameTab );
    _tab_menu->addSeparator();
}

void TabbedPlotWidget::setSiblingsList(const std::map<QString, TabbedPlotWidget *> &other_tabbed_widgets)
{
    _other_siblings = other_tabbed_widgets;
}

PlotMatrix *TabbedPlotWidget::currentTab()
{
    return static_cast<PlotMatrix*>( ui->tabWidget->currentWidget() );
}

QTabWidget *TabbedPlotWidget::tabWidget()
{
    return ui->tabWidget;
}

void TabbedPlotWidget::addTab( PlotMatrix* tab)
{
    if( !tab )
    {
        tab = new PlotMatrix("plot", _mapped_data, this);
        ui->tabWidget->addTab( tab, QString("plot") );
        tab->addColumn();

        connect( tab, SIGNAL(plotAdded(PlotWidget*)), _main_window, SLOT(on_plotAdded(PlotWidget*)));
        connect( tab, SIGNAL(layoutModified()),       _main_window, SLOT( on_undoableChange()) );
    }
    else{
        ui->tabWidget->addTab( tab, tab->name() );
    }

    ui->tabWidget->setCurrentWidget( tab );

    tab->setHorizontalLink( _horizontal_link );

    //TODO  grid->setActiveTracker( ui->pushButtonActivateTracker->isChecked() );

    emit undoableChangeHappened();
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
            this->addTab( NULL );
            num_tabs++;
        }
        PlotMatrix* plot_matrix = static_cast<PlotMatrix*>(  ui->tabWidget->widget(index) );
        bool success = plot_matrix->xmlLoadState( plotmatrix_el );

        // read tab name
        if( plotmatrix_el.hasAttribute("tab_name"))
        {
            QString tab_name = plotmatrix_el.attribute("tab_name" );
            ui->tabWidget->setTabText( index, tab_name );
            plot_matrix->setName( tab_name );
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

void TabbedPlotWidget::renameCurrentTab()
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
        currentTab()->setName( newName );
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
    addTab( NULL );
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushremoveEmpty_pressed()
{
    PlotMatrix *tab = currentTab();

    for( int row = 0; row< tab->numRows(); row++)
    {
        while( tab->numRows() > 1 &&
               tab->isRowEmpty( row ) &&
               row < tab->numRows() )
        {
            tab->removeRow( row );
        }
    }

    for( int col = 0; col< tab->numColumns(); col++)
    {
        while( tab->numColumns() > 1 &&
               tab->isColumnEmpty( col ) &&
               col < tab->numColumns() )
        {
            tab->removeColumn( col );
        }
    }

    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_tabWidget_currentChanged(int index)
{
    if( ui->tabWidget->count() == 0)
    {
        if( _parent_type.compare("main_window") == 0)
        {
            addTab( NULL);
        }
        else{
            this->parent()->deleteLater();
        }
    }

    PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
    if( tab )
    {
        tab->replot();
    }
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

void TabbedPlotWidget::on_requestTabMovement(const QString & destination_name)
{
    TabbedPlotWidget* destination_widget = _other_siblings[destination_name];

    PlotMatrix* tab_to_move = currentTab();
    int index = ui->tabWidget->tabBar()->currentIndex ();

    const QString& tab_name =  this->tabWidget()->tabText(index);

    destination_widget->tabWidget()->addTab( tab_to_move, tab_name );

    // tab_to_move->setParent( destination_widget );

    qDebug() << "move "<< tab_name<< " into " << destination_name;

}

void TabbedPlotWidget::moveTabIntoNewWindow()
{
    emit sendTabToNewWindow( currentTab() );
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
                QMenu* submenu = new QMenu("Move tab to...");
                _tab_menu->addMenu( submenu );

                std::map<QString,TabbedPlotWidget*>::iterator it;
                QSignalMapper* signalMapper = new QSignalMapper(submenu);

                //-----------------------------------
                QAction* action_new_window = submenu->addAction( "New Window" );

                QIcon icon;
                icon.addFile(QStringLiteral(":/icons/resources/stacks_32px.png"), QSize(16, 16), QIcon::Normal, QIcon::Off);

                action_new_window->setIcon( icon);
                submenu->addSeparator();

                connect( action_new_window, SIGNAL(triggered()), this, SLOT(moveTabIntoNewWindow() ));

                //-----------------------------------
                for ( it = _other_siblings.begin(); it != _other_siblings.end(); it++)
                {
                    QString name = it->first;
                    TabbedPlotWidget* tabbed_menu = it->second;
                    if( tabbed_menu != this )
                    {
                        QAction* action = submenu->addAction( name );
                        connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
                        signalMapper->setMapping( action, name );
                    }
                }

                connect(signalMapper, SIGNAL(mapped(const QString &)),
                        this, SLOT(on_requestTabMovement(const QString &)));

                //-------------------------------
                _tab_menu->exec( mouse_event->globalPos() );
                //-------------------------------
                submenu->deleteLater();
            }
        }
    }

    // Standard event processing
    return QObject::eventFilter(obj, event);
}
