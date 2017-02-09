#ifndef RULE_EDITING_H
#define RULE_EDITING_H

#include <QDialog>

namespace Ui {
class RuleEditing;
}

class RuleEditing : public QDialog
{
  Q_OBJECT

public:
  explicit RuleEditing(QWidget *parent = 0);
  ~RuleEditing();

private:
  Ui::RuleEditing *ui;
};

#endif // RULE_EDITING_H
