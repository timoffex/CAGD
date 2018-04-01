
#include "beziereditor.h"

#include <QApplication>


int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    BezierEditor editor;
    editor.show();

    return app.exec();
}
