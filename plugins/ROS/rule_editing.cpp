#include "rule_editing.h"
#include "ui_rule_editing.h"

RuleEditing::RuleEditing(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::RuleEditing)
{
  ui->setupUi(this);
}

RuleEditing::~RuleEditing()
{
  delete ui;
}
