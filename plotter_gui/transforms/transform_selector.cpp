#include "transform_selector.h"
#include "ui_transform_selector.h"

transform_selector::transform_selector(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::transform_selector)
{
    ui->setupUi(this);
}

transform_selector::~transform_selector()
{
    delete ui;
}
