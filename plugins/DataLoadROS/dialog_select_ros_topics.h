#ifndef DIALOG_SELECT_ROS_TOPICS_H
#define DIALOG_SELECT_ROS_TOPICS_H

#include <QDialog>
#include <QString>
#include <QFile>
#include <QStringList>

namespace Ui {
class dialogSelectRosTopics;
}

class DialogSelectRosTopics : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSelectRosTopics(QStringList topic_list, QWidget *parent = 0);
    ~DialogSelectRosTopics();

    QStringList getSelectedItems();

    QString getLoadedRules() const;

private slots:

    void on_pushButtonLoadRules_pressed();


    void on_buttonBox_accepted();

    void on_listRosTopics_itemSelectionChanged();

    void on_checkBoxEnableSubstitution_toggled(bool checked);

private:

    void readRuleFile(QFile& file);

    QString _loaded_rules;

    QStringList _topic_list;

    Ui::dialogSelectRosTopics *ui;
};

#endif // DIALOG_SELECT_ROS_TOPICS_H
