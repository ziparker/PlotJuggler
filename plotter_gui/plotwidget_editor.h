#ifndef PLOTWIDGET_EDITOR_H
#define PLOTWIDGET_EDITOR_H

#include <QDialog>
#include "plotwidget.h"
#include "color_wheel.hpp"
#include "color_preview.hpp"

namespace Ui {
class plotwidget_editor;
}

class PlotwidgetEditor : public QDialog
{
  Q_OBJECT

public:
  explicit PlotwidgetEditor(PlotWidget* plotwidget, QWidget *parent = nullptr);
  ~PlotwidgetEditor();

public slots:
  void onColorChanged(QColor c);

private slots:

  void on_editColotText_textChanged(const QString &arg1);

  void on_radioLines_toggled(bool checked);

  void on_radioPoints_toggled(bool checked);

  void on_radioBoth_toggled(bool checked);

  void on_buttonBox_accepted();

  void on_buttonBox_rejected();

  void on_checkBoxMax_toggled(bool checked);

  void on_checkBoxMin_toggled(bool checked);

  void on_tableWidget_itemSelectionChanged();

  void on_tableWidget_cellClicked(int row, int column);

  void on_listWidget_currentRowChanged(int currentRow);

private:
  Ui::plotwidget_editor *ui;

  color_widgets::ColorWheel* _color_wheel;
  color_widgets::ColorPreview* _color_preview;
  PlotWidget* _plotwidget;
  PlotWidget* _plotwidget_origin;

  void setupColorWidget();
  void setupTable();
};

#endif // PLOTWIDGET_EDITOR_H
