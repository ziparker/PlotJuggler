#ifndef TRANSFORM_SELECTOR_H
#define TRANSFORM_SELECTOR_H

#include <QDialog>

namespace Ui {
class transform_selector;
}

class TransformSelector : public QDialog
{
    Q_OBJECT

public:
    TransformSelector(QString* default_tansform,
                      std::map<std::string, QString> *curve_transforms,
                      QWidget *parent = nullptr);
    ~TransformSelector();

private slots:

    void on_buttonApplyDefault_clicked();

    void on_buttonResetAll_clicked();

    void on_transform_selector_accepted();

private:
    Ui::transform_selector *ui;

    std::map<std::string, QString> *_curves_trans;
};

#endif // TRANSFORM_SELECTOR_H
