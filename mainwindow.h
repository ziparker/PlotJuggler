#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "draggablewidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void addPlotWidget(QString name, int row, int col);
private slots:

    void swapWidgets(QString src, QString dst);

    void on_splitter_splitterMoved(int, int);
    void resizeEvent(QResizeEvent *) Q_DECL_OVERRIDE;

    void on_lineEdit_textChanged(const QString &arg1);

    void on_radioRegExp_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

    void on_pushAddRow_pressed();

    void on_pushAddColumn_pressed();

    void createActions();

    void deletePlot();

private:
    Ui::MainWindow *ui;

    QList<DragableWidget*> plotWidgets;

    QAction *deleteOneAct;
    QAction *deleteAllAct;

protected:
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void contextMenuEvent(QContextMenuEvent *event) Q_DECL_OVERRIDE;

};

#endif // MAINWINDOW_H
