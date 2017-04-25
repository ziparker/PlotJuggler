#ifndef AXIS_LIMITS_DIALOG_H
#define AXIS_LIMITS_DIALOG_H

#include <QDialog>
#include <QRadialGradient>
#include <PlotJuggler/plotdata.h>

namespace Ui {
class AxisLimitsDialog;
}

class AxisLimitsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AxisLimitsDialog(QWidget *parent = 0);
    ~AxisLimitsDialog();

    void setDefaultRange(PlotData::RangeValue range);

     PlotData::RangeValue rangeY() const { return _limits; }

private slots:
    void on_checkBoxMinY_toggled(bool checked);

    void on_checkBoxMaxY_toggled(bool checked);

    void on_pushButtonDone_pressed();

    void on_pushButtonMinY_pressed();

    void on_pushButtonMaxY_pressed();

private:
    Ui::AxisLimitsDialog *ui;

    PlotData::RangeValue _parent_limits;

    PlotData::RangeValue _limits;

};

#endif // AXIS_LIMITS_DIALOG_H
