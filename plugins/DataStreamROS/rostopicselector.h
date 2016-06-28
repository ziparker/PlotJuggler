#ifndef ROSTOPICSELECTOR_H
#define ROSTOPICSELECTOR_H

#include <QDialog>
#include <QStringList>

namespace Ui {
class RosTopicSelector;
}

class RosTopicSelector : public QDialog
{
    Q_OBJECT

public:
    explicit RosTopicSelector(QWidget *parent = 0);
    ~RosTopicSelector();

    bool connectionSuccesfull();

    QStringList getSelectedTopicsList();

private slots:
    void on_buttonConnect_pressed();

    void on_buttonDisconnect_pressed();

    void on_checkBoxEnvironmentSettings_toggled(bool checked);

    void showNoMasterMessage();

    void on_listTopics_itemSelectionChanged();

    void on_buttonBox_accepted();

private:
    Ui::RosTopicSelector *ui;

    QStringList _selected_topics;
};

#endif // ROSTOPICSELECTOR_H
