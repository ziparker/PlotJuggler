#ifndef TRANSFORM_SELECTOR_H
#define TRANSFORM_SELECTOR_H

#include <QDialog>

namespace Ui {
class transform_selector;
}

class transform_selector : public QDialog
{
    Q_OBJECT

public:
    explicit transform_selector(QWidget *parent = nullptr);
    ~transform_selector();

private:
    Ui::transform_selector *ui;
};

#endif // TRANSFORM_SELECTOR_H
