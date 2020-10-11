#include "protocol_selector.h"
#include "ui_protocol_selector.h"

ProtocolSelector::ProtocolSelector(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ProtocolSelector)
{
  ui->setupUi(this);
}

ProtocolSelector::~ProtocolSelector()
{
  delete ui;
}
