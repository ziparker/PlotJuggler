#ifndef AddCustomPlotDialog_H
#define AddCustomPlotDialog_H

#include <QDialog>
#include <QListWidgetItem>
#include <qwt_plot_curve.h>
#include <unordered_map>
#include "PlotJuggler/plotdata.h"
#include "custom_function.h"
#include "ui_function_editor.h"


class AddCustomPlotDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddCustomPlotDialog(PlotDataMapRef &plotMapData,
                               const std::vector<CustomPlotPtr>& mapped_custom_plots,
                               QWidget *parent);
    virtual ~AddCustomPlotDialog() override;

    void setLinkedPlotName(const QString &linkedPlotName);
    virtual void accept() override;

    QString getLinkedData() const;
    QString getGlobalVars() const;
    QString getEquation() const;
    QString getName() const;
    const PlotData& getPlotData() const;
    void editExistingPlot(CustomPlotPtr data);
    CustomPlotPtr getCustomPlotData() const;

private slots:

    void on_curvesListWidget_doubleClicked(const QModelIndex &index);

    void on_snippetsListSaved_currentRowChanged(int currentRow);

    void on_snippetsListSaved_doubleClicked(const QModelIndex &index);

    void on_snippetsListRecent_currentRowChanged(int currentRow);

    void on_snippetsListRecent_doubleClicked(const QModelIndex &index);

    void on_nameLineEdit_textChanged(const QString &arg1);

    void recentContextMenu(const QPoint &pos);

    void savedContextMenu(const QPoint &pos);

    void on_buttonLoadFunctions_clicked();

    void on_buttonSaveFunctions_clicked();

    void on_pushButtonSave_clicked();

    void onRenameSaved();

private:
    void importSnippets(const QByteArray &xml_text);

    QByteArray exportSnippets() const;


    PlotDataMapRef &_plot_map_data;
    const std::vector<CustomPlotPtr> &_custom_plots;
    Ui::FunctionEditor *ui;

    CustomPlotPtr _plot;
    bool _is_new;

    std::map<QString, SnippetData> _snipped_saved;
    std::map<QString, SnippetData> _snipped_recent;
};

#endif // AddCustomPlotDialog_H
