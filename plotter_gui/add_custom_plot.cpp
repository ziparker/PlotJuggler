#include "add_custom_plot.h"
#include "ui_add_custom_plot.h"
#include <QDebug>
#include <QMessageBox>
#include <QFont>
#include <QDomDocument>
#include <QDomElement>
#include <QFontDatabase>
#include <QFile>
#include <QMenu>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QByteArray>
#include <QInputDialog>
#include "custom_plot.h"
#include "plotwidget.h"

AddCustomPlotDialog::AddCustomPlotDialog(PlotDataMapRef &plotMapData,
                                     const std::vector<CustomPlotPtr> &mapped_custom_plots,
                                     QWidget *parent) :
    QDialog(parent),
    _plot_map_data(plotMapData),
    _custom_plots(mapped_custom_plots),
    ui(new Ui::AddCustomPlotDialog)
{
    ui->setupUi(this);

    this->setWindowTitle("Create a custom timeseries (EXPERIMENTAL)");

    const QFont fixedFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    ui->globalVarsTextField->setFont(fixedFont);
    ui->mathEquation->setFont(fixedFont);
    ui->snippetTextEdit->setFont(fixedFont);

    QStringList numericPlotNames;
    for(const auto &p : _plot_map_data.numeric)
    {
        QString name = QString::fromStdString(p.first);
        numericPlotNames.push_back(name);
    }
    numericPlotNames.sort(Qt::CaseInsensitive);
    for(const QString& name : numericPlotNames)
    {
        ui->linkedChannelCombobox->addItem(name);
        ui->curvesListWidget->addItem(name);
    }

    QSettings settings;
    QByteArray saved_xml = settings.value("AddCustomPlotDialog.savedXML", QByteArray() ).toByteArray();
    restoreGeometry(settings.value("AddCustomPlotDialog.geometry").toByteArray());

    if( saved_xml.isEmpty() )
    {
        QFile file("://resources/snippets.xml");
        if(!file.open(QIODevice::ReadOnly))
        {
            throw std::runtime_error( "problem with snippets.xml" );
        }
        saved_xml = file.readAll();
    }

    importSnippets(saved_xml);

    ui->snippetsListRecent->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui->snippetsListRecent, &QListWidget::customContextMenuRequested,
            this,&AddCustomPlotDialog::recentContextMenu );

    ui->snippetsListSaved->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui->snippetsListSaved, &QListWidget::customContextMenuRequested,
            this, &AddCustomPlotDialog::savedContextMenu );

    ui->splitter->setStretchFactor(0,3);
    ui->splitter->setStretchFactor(1,2);
}

AddCustomPlotDialog::~AddCustomPlotDialog()
{
    QSettings settings;
    settings.setValue("AddCustomPlotDialog.savedXML", exportSnippets() );
    settings.setValue("AddCustomPlotDialog.geometry", saveGeometry());
    delete ui;
}

void AddCustomPlotDialog::setLinkedPlotName(const QString &linkedPlotName)
{
    ui->linkedChannelCombobox->setCurrentText(linkedPlotName);
}

void AddCustomPlotDialog::accept()
{
    try {
        std::string plotName = getName().toStdString();
        if(_is_new)
        {
            // check if name is unique
            if(_plot_map_data.numeric.count(plotName) != 0)
            {
                throw std::runtime_error("plot name already exists");
            }
        }
        _plot = std::make_shared<CustomPlot>(getLinkedData().toStdString(),
                                           plotName,
                                           getGlobalVars(),
                                           getEquation());
        QDialog::accept();
    }
    catch (const std::runtime_error &e) {
        _plot.reset();
        QMessageBox::critical(this, "Error", "Failed to create math plot : "
                              + QString::fromStdString(e.what()));
    }
}

QString AddCustomPlotDialog::getLinkedData() const
{
    return ui->linkedChannelCombobox->currentText();
}

QString AddCustomPlotDialog::getGlobalVars() const
{
    return ui->globalVarsTextField->toPlainText();
}

QString AddCustomPlotDialog::getEquation() const
{
    return ui->mathEquation->toPlainText();
}

QString AddCustomPlotDialog::getName() const
{
    return ui->nameLineEdit->text();
}


void AddCustomPlotDialog::editExistingPlot(CustomPlotPtr data)
{
    ui->globalVarsTextField->setPlainText(data->globalVars());
    ui->mathEquation->setPlainText(data->function());
    setLinkedPlotName(QString::fromStdString(data->linkedPlotName()));
    ui->pushButtonCreate->setText("Update");
    ui->nameLineEdit->setText(QString::fromStdString(data->name()));
    ui->nameLineEdit->setEnabled(false);

    _is_new = false;
}

CustomPlotPtr AddCustomPlotDialog::getCustomPlotData() const
{
    return _plot;
}

void AddCustomPlotDialog::on_curvesListWidget_doubleClicked(const QModelIndex &index)
{
    QString appendString = QString("$$%1$$").arg(ui->curvesListWidget->item(index.row())->text());
    if(ui->globalVarsTextField->hasFocus())
    {
        ui->globalVarsTextField->insertPlainText(appendString);
    }
    else if(ui->mathEquation->hasFocus())
    {
        ui->mathEquation->insertPlainText(appendString);
    }
}

void AddCustomPlotDialog::importSnippets(const QByteArray& xml_text)
{
    _snipped_saved.clear();
    ui->snippetsListSaved->clear();

    QDomDocument doc;
    QString parseErrorMsg;
    int parseErrorLine;
    if(!doc.setContent(xml_text, &parseErrorMsg, &parseErrorLine))
    {
        QMessageBox::critical(nullptr, "Error",
                              QString("Failed to parse snippets.xml, error %1 at line %2")
                              .arg(parseErrorMsg).arg(parseErrorLine));
    }
    else
    {
        QDomElement docElem = doc.documentElement();
        QDomNode n = docElem.firstChild();
        while(!n.isNull()) {
            QDomElement e = n.toElement(); // try to convert the node to an element.
            if(!e.isNull() && e.tagName() == "snippet") {
                SnippetData snippet;
                snippet.name = e.attribute("name");
                snippet.globalVars = e.firstChildElement("global").text().trimmed();
                snippet.equation = e.firstChildElement("equation").text().trimmed();
                _snipped_saved.insert( {snippet.name, snippet } );
            }
            n = n.nextSibling();
        }
    }
    for(const auto &it : _snipped_saved)
    {
        ui->snippetsListSaved->addItem( it.first );
    }

    for( const auto& math_plot: _custom_plots)
    {
        SnippetData snippet;
        snippet.name = QString::fromStdString(math_plot->name());

        if( _snipped_saved.count( snippet.name ) > 0 )
        {
            continue;
        }

        snippet.globalVars = math_plot->globalVars();
        snippet.equation = math_plot->function();
        _snipped_recent.insert( {snippet.name, snippet } );
        ui->snippetsListRecent->addItem( snippet.name );
    }
    ui->snippetsListRecent->sortItems();
    ui->snippetsListSaved->sortItems();
}

QByteArray AddCustomPlotDialog::exportSnippets() const
{
    QDomDocument doc;
    auto root = doc.createElement("snippets");
    doc.appendChild( root );

    for (const auto& it: _snipped_saved )
    {
        const auto& snippet = it.second;

        auto element = doc.createElement("snippet");
        element.setAttribute("name", it.first);

        auto global_el = doc.createElement("global");
        global_el.appendChild( doc.createTextNode( snippet.globalVars ) );

        auto equation_el = doc.createElement("equation");
        equation_el.appendChild( doc.createTextNode( snippet.equation ) );

        element.appendChild( global_el );
        element.appendChild( equation_el );
        root.appendChild( element );
    }
    return doc.toByteArray(2);
}


void AddCustomPlotDialog::on_snippetsListSaved_currentRowChanged(int current_row)
{
    if( current_row < 0 )
    {
        ui->snippetTextEdit->setPlainText("");
        return;
    }
    const auto& name = ui->snippetsListSaved->currentItem()->text();
    const SnippetData& snippet = _snipped_saved.at(name);
    QString desc = QString("%1\n\nfunction calc(time,value)\n{\n%2\n}").arg(snippet.globalVars).arg(snippet.equation);
    ui->snippetTextEdit->setPlainText(desc);
}

void AddCustomPlotDialog::on_snippetsListSaved_doubleClicked(const QModelIndex &index)
{
    const auto& name = ui->snippetsListSaved->item( index.row() )->text();
    const SnippetData& snippet = _snipped_saved.at(name);

    ui->globalVarsTextField->setPlainText(snippet.globalVars);
    ui->mathEquation->setPlainText(snippet.equation);
}

void AddCustomPlotDialog::on_snippetsListRecent_currentRowChanged(int current_row)
{
    if( current_row < 0 )
    {
        ui->snippetTextEdit->setPlainText("");
        return;
    }
    const auto& name = ui->snippetsListRecent->currentItem()->text();
    const SnippetData& snippet = _snipped_recent.at(name);

    QString desc = QString("%1\n\nfunction calc(time,value)\n{\n%2\n}")
            .arg(snippet.globalVars)
            .arg(snippet.equation);
    ui->snippetTextEdit->setPlainText(desc);
}

void AddCustomPlotDialog::on_snippetsListRecent_doubleClicked(const QModelIndex &index)
{
    const auto& name = ui->snippetsListSaved->item( index.row() )->text();
    const SnippetData& snippet = _snipped_saved.at(name);

    ui->globalVarsTextField->setPlainText(snippet.globalVars);
    ui->mathEquation->setPlainText(snippet.equation);
}

void AddCustomPlotDialog::recentContextMenu(const QPoint &pos)
{
    auto list_recent = ui->snippetsListRecent;

    if( list_recent->selectedItems().size() != 1)
    {
        return;
    }

    auto list_saved = ui->snippetsListSaved;

    QMenu menu;

    menu.addAction("Move to Saved", this, [list_recent, list_saved, this]()
    {
        auto item = list_recent->selectedItems().first();
        const auto& name = item->text();

        _snipped_saved.insert( {name, _snipped_recent.at(name) } );
        list_saved->addItem( name );
        list_saved->sortItems();

        _snipped_recent.erase( name );
        delete list_recent->takeItem( list_recent->row(item) );
    });
    menu.exec( list_recent->mapToGlobal(pos) );
}

void AddCustomPlotDialog::savedContextMenu(const QPoint &pos)
{
    auto list_saved = ui->snippetsListSaved;

    if( list_saved->selectedItems().size() != 1)
    {
        return;
    }

    QMenu menu;

    menu.addAction("Rename...",  this, [this, list_saved]()
    {
        auto item = list_saved->selectedItems().first();
        const auto& name = item->text();

        bool ok;
        QString new_name = QInputDialog::getText(this, tr("Change the name of the function"),
                                             tr("New name:"), QLineEdit::Normal,
                                             name, &ok);

        if (!ok || new_name.isEmpty() || new_name == name)
        {
            return;
        }

        SnippetData snippet = _snipped_saved[ name ];
        _snipped_saved.erase( name );
        snippet.name = new_name;

        _snipped_saved.insert( {new_name, snippet} );
        item->setText( new_name );
        ui->snippetsListSaved->sortItems();
    });

    menu.addAction("Remove",  this, [this, list_saved]()
    {
        const auto& item = list_saved->selectedItems().first();
        _snipped_saved.erase( item->text() );
        delete list_saved->takeItem( list_saved->row(item) );
    });

    menu.exec( list_saved->mapToGlobal(pos) );
}

void AddCustomPlotDialog::on_nameLineEdit_textChanged(const QString &arg1)
{
    ui->pushButtonCreate->setEnabled( !arg1.isEmpty() );
    ui->pushButtonSave->setEnabled(   !arg1.isEmpty() );
}

void AddCustomPlotDialog::on_buttonLoadFunctions_clicked()
{
    QSettings settings;
    QString directory_path  = settings.value("AddCustomPlotDialog.loadDirectory",
                                             QDir::currentPath() ).toString();

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Snippet Library"),
                                                    directory_path,
                                                    tr("Snippets (*.snippets.xml)"));
    if (fileName.isEmpty()){
        return;
    }

    QFile file(fileName);

    if ( !file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "Error", QString("Failed to open the file [%1]").arg(fileName) );
        return;
    }

    directory_path = QFileInfo(fileName).absolutePath();
    settings.setValue("AddCustomPlotDialog.loadDirectory", directory_path );

    importSnippets( file.readAll() );
}

void AddCustomPlotDialog::on_buttonSaveFunctions_clicked()
{
    QSettings settings;
    QString directory_path  = settings.value("AddCustomPlotDialog.loadDirectory",
                                             QDir::currentPath() ).toString();

    QString fileName = QFileDialog::getSaveFileName(this, tr("Open Snippet Library"),
                                                    directory_path,
                                                    tr("Snippets (*.snippets.xml)"));

    if (fileName.isEmpty()){
        return;
    }
    if( !fileName.endsWith(".snippets.xml") )
    {
        fileName.append(".snippets.xml");
    }

    QFile file(fileName);
    if ( !file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, "Error", QString("Failed to open the file [%1]").arg(fileName));
        return;
    }
    auto data = exportSnippets();

    file.write(data);
    file.close();
}

void AddCustomPlotDialog::on_pushButtonSave_clicked()
{
    QString name = ui->nameLineEdit->text();

    SnippetData snippet;
    snippet.name = name;
    snippet.globalVars = ui->globalVarsTextField->toPlainText();
    snippet.equation   = ui->mathEquation->toPlainText();

    if( _snipped_saved.count(name) )
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning");
        msgBox.setText(tr("A function with the same name exists already in the list of saved functions.\n"));
        msgBox.addButton(QMessageBox::Cancel);
        QPushButton* button = msgBox.addButton(tr("Overwrite"), QMessageBox::YesRole);
        msgBox.setDefaultButton(button);

        int res = msgBox.exec();

        if( res < 0 ||  res == QMessageBox::Cancel) {
            return;
        }
    }
    else{
        ui->snippetsListSaved->addItem(name);
    }
    _snipped_saved[name] = snippet;
    ui->snippetsListSaved->sortItems();

    on_snippetsListSaved_currentRowChanged( ui->snippetsListSaved->currentRow() );
}
