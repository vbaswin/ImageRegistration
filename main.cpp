#include "mainwindow.h"

#include <QApplication>
#include <QDebug>
#include <QFile>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QFile styleFile(":/styles/dark_blue.qss");
    if (styleFile.open(QFile::ReadOnly)) {
        QString styleSheet = QLatin1String(styleFile.readAll());
        a.setStyleSheet(styleSheet);
        styleFile.close();
    } else {
        qWarning() << "Failed to load stylesheet!";
    }
    MainWindow w;
    w.show();
    return a.exec();
}
