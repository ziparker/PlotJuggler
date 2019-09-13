#ifndef RANDOM_COLOR_H
#define RANDOM_COLOR_H

#include <QColor>

inline QColor randomColorHint()
{
    static int index = 0;
    QColor color;
   // https://matplotlib.org/3.1.1/users/dflt_style_changes.html
    switch( index%10 )
    {
    case 0:  color =  QColor("#1950ff");   break;
    case 1:  color =  QColor("#ff7c00");   break;
    case 2:  color =  QColor("#1ac938");   break;
    case 3:  color =  QColor("#e81721");   break;
    case 4:  color =  QColor("#9139e2");   break;

    case 5:  color =  QColor("#00d7ff");   break;
    case 6:  color =  QColor("#f14cc1");   break;
    case 7:  color =  QColor("#ffc400");   break;
    case 8:  color =  QColor("#9f4800");   break;
    case 9:  color =  QColor("#a3a3a3");   break;
    }
    index++;
    return color;
}

#endif // RANDOM_COLOR_H
