#ifndef PROTOCOL_SELECTOR_H
#define PROTOCOL_SELECTOR_H

#include <QWidget>

namespace Ui {
class ProtocolSelector;
}

class ProtocolSelector : public QWidget
{
  Q_OBJECT

public:
  explicit ProtocolSelector(QWidget *parent = nullptr);
  ~ProtocolSelector();

private:
  Ui::ProtocolSelector *ui;
};

#endif // PROTOCOL_SELECTOR_H
