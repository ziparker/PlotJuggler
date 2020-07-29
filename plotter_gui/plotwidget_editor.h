#ifndef PLOTWIDGET_EDITOR_H
#define PLOTWIDGET_EDITOR_H

#include <QDialog>
#include <QKeyEvent>
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

  void on_checkBoxMax_toggled(bool checked);

  void on_checkBoxMin_toggled(bool checked);

  void on_listWidget_currentRowChanged(int currentRow);

  void on_pushButtonReset_clicked();

  void on_lineLimitMax_editingFinished();

  void on_lineLimitMin_editingFinished();

  void on_pushButtonCancel_pressed();

  void on_pushButtonSave_pressed();

private:
  Ui::plotwidget_editor *ui;

  color_widgets::ColorWheel* _color_wheel;
  color_widgets::ColorPreview* _color_preview;
  PlotWidget* _plotwidget;
  PlotWidget* _plotwidget_origin;

  void setupColorWidget();
  void setupTable();
  void updateLimits();
};

#endif // PLOTWIDGET_EDITOR_H
