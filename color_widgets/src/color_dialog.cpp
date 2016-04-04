/**
 * \file
 *
 * \author Mattia Basaglia
 *
 * \copyright Copyright (C) 2013-2015 Mattia Basaglia
 * \copyright Copyright (C) 2014 Calle Laakkonen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "color_dialog.hpp"
#include "ui_color_dialog.h"

#include <QDropEvent>
#include <QDragEnterEvent>
#include <QDesktopWidget>
#include <QMimeData>
#include <QPushButton>
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
#include <QScreen>
#endif
namespace color_widgets {

class ColorDialog::Private
{
public:
    Ui_ColorDialog ui;
    ButtonMode button_mode;
    bool pick_from_screen;


    Private() : pick_from_screen(false)
    {}

};

ColorDialog::ColorDialog(QWidget *parent, Qt::WindowFlags f) :
    QDialog(parent, f), p(new Private)
{
    p->ui.setupUi(this);

    setAcceptDrops(true);

    // Add "pick color" button
    QPushButton *pickButton = p->ui.buttonBox->addButton(tr("Pick"), QDialogButtonBox::ActionRole);
    pickButton->setIcon(QIcon::fromTheme("color-picker"));

    setButtonMode(OkApplyCancel);

    connect(p->ui.wheel,SIGNAL(displayFlagsChanged(ColorWheel::DisplayFlags)),SIGNAL(wheelFlagsChanged(ColorWheel::DisplayFlags)));
}

QSize ColorDialog::sizeHint() const
{
    return QSize(400,0);
}

ColorWheel::DisplayFlags ColorDialog::wheelFlags() const
{
    return p->ui.wheel->displayFlags();
}

QColor ColorDialog::color() const
{
    QColor col = p->ui.wheel->color();
    return col;
}

void ColorDialog::setColor(const QColor &c)
{
    p->ui.preview->setComparisonColor(c);
    setColorInternal(c);
}

void ColorDialog::setColorInternal(const QColor &c)
{
    /**
     * \note Unlike setColor, this is used to update the current color which
     * migth differ from the final selected color
     */
    p->ui.wheel->setColor(c);
    update_widgets();
}

void ColorDialog::showColor(const QColor &c)
{
    setColor(c);
    show();
}

void ColorDialog::setWheelFlags(ColorWheel::DisplayFlags flags)
{
    p->ui.wheel->setDisplayFlags(flags);
}

void ColorDialog::setPreviewDisplayMode(ColorPreview::DisplayMode mode)
{
    p->ui.preview->setDisplayMode(mode);
}

ColorPreview::DisplayMode ColorDialog::previewDisplayMode() const
{
    return p->ui.preview->displayMode();
}


void ColorDialog::setButtonMode(ButtonMode mode)
{
    p->button_mode = mode;
    QDialogButtonBox::StandardButtons btns;
    switch(mode) {
        case OkCancel: btns = QDialogButtonBox::Ok | QDialogButtonBox::Cancel; break;
        case OkApplyCancel: btns = QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::Apply | QDialogButtonBox::Reset; break;
        case Close: btns = QDialogButtonBox::Close;
    }
    p->ui.buttonBox->setStandardButtons(btns);
}

ColorDialog::ButtonMode ColorDialog::buttonMode() const
{
    return p->button_mode;
}

void ColorDialog::update_widgets()
{
    bool blocked = signalsBlocked();
    blockSignals(true);
    foreach(QWidget* w, findChildren<QWidget*>())
        w->blockSignals(true);

    QColor col = color();
    p->ui.preview->setColor(col);

    blockSignals(blocked);
    foreach(QWidget* w, findChildren<QWidget*>())
        w->blockSignals(false);

    emit colorChanged(col);
}

void ColorDialog::set_hsv()
{
    if ( !signalsBlocked() )
    {
        update_widgets();
    }
}

void ColorDialog::set_rgb()
{
    if ( !signalsBlocked() )
    {
     //   p->ui.wheel->setColor(col);
        update_widgets();
    }
}



void ColorDialog::on_buttonBox_clicked(QAbstractButton *btn)
{
    QDialogButtonBox::ButtonRole role = p->ui.buttonBox->buttonRole(btn);

    switch(role) {
    case QDialogButtonBox::AcceptRole:
    case QDialogButtonBox::ApplyRole:
        // Explicitly select the color
        p->ui.preview->setComparisonColor(color());
        emit colorSelected(color());
        break;

    case QDialogButtonBox::ActionRole:
        // Currently, the only action button is the "pick color" button
        grabMouse(Qt::CrossCursor);
        p->pick_from_screen = true;
        break;

    case QDialogButtonBox::ResetRole:
        // Restore old color
        setColorInternal(p->ui.preview->comparisonColor());
        break;

    default: break;
    }
}

void ColorDialog::dragEnterEvent(QDragEnterEvent *event)
{
    if ( event->mimeData()->hasColor() ||
         ( event->mimeData()->hasText() && QColor(event->mimeData()->text()).isValid() ) )
        event->acceptProposedAction();
}


void ColorDialog::dropEvent(QDropEvent *event)
{
    if ( event->mimeData()->hasColor() )
    {
        setColorInternal(event->mimeData()->colorData().value<QColor>());
        event->accept();
    }
    else if ( event->mimeData()->hasText() )
    {
        QColor col(event->mimeData()->text());
        if ( col.isValid() )
        {
            setColorInternal(col);
            event->accept();
        }
    }
}

static QColor get_screen_color(const QPoint &global_pos)
{
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
    WId id = QApplication::desktop()->winId();
    QImage img = QPixmap::grabWindow(id, global_pos.x(), global_pos.y(), 1, 1).toImage();
#else
    int screenNum = QApplication::desktop()->screenNumber(global_pos);
    QScreen *screen = QApplication::screens().at(screenNum);

    WId wid = QApplication::desktop()->winId();
    QImage img = screen->grabWindow(wid, global_pos.x(), global_pos.y(), 1, 1).toImage();
#endif

    return img.pixel(0,0);
}

void ColorDialog::mouseReleaseEvent(QMouseEvent *event)
{
    if (p->pick_from_screen)
    {
        setColorInternal(get_screen_color(event->globalPos()));
        p->pick_from_screen = false;
        releaseMouse();
    }
}

void ColorDialog::mouseMoveEvent(QMouseEvent *event)
{
    if (p->pick_from_screen)
    {
        setColorInternal(get_screen_color(event->globalPos()));
    }
}

} // namespace color_widgets
