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

private slots:
    void on_pushButton_2_pressed();
    void on_pushButton_pressed();

    void swapWidgets(QString src, QString dst);

    void on_splitter_splitterMoved(int, int);
    void resizeEvent(QResizeEvent *) Q_DECL_OVERRIDE;

    void on_lineEdit_textChanged(const QString &arg1);


    void on_radioRegExp_toggled(bool checked);

    void on_checkBoxCaseSensitive_toggled(bool checked);

private:
    Ui::MainWindow *ui;

    QList<DragableWidget*> editors;

protected:
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

};

#endif // MAINWINDOW_H
