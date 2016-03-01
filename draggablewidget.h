#ifndef DragableWidget_H
#define DragableWidget_H

#include <QObject>
#include <QTextEdit>

class DragableWidget : public QTextEdit
{
    Q_OBJECT

public:
    DragableWidget(QWidget *parent=0);

protected:
    void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
    void dragMoveEvent(QDragMoveEvent *event) Q_DECL_OVERRIDE;
    void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

signals:
    void swapWidgets(QString s, QString to);

};

#endif // DragableWidget_H
