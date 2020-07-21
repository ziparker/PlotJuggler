#ifndef PLOTWIDGET_EDITOR_H
#define PLOTWIDGET_EDITOR_H

#include <QDialog>
#include "plotwidget.h"

namespace Ui {
class plotwidget_editor;
}

class PlotwidgetEditor : public QDialog
{
  Q_OBJECT

public:
  explicit PlotwidgetEditor(PlotWidget* plotwidget, QWidget *parent = nullptr);
  ~PlotwidgetEditor();

private:
  Ui::plotwidget_editor *ui;
};

#endif // PLOTWIDGET_EDITOR_H
