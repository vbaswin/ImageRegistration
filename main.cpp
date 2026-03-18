#include "mainwindow.h"

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

int main(int argc, char *argv[])
{
    // Essential for vtkGPUVolumeRayCastMapper when using QVTKOpenGLNativeWidget:
    // 1. Disable Multi-Sampling (MSAA) which interferes with VTK's offscreen volume rendering passes.
    QSurfaceFormat format = QVTKOpenGLNativeWidget::defaultFormat();
    format.setSamples(0);
    QSurfaceFormat::setDefaultFormat(format);

    // 2. Share OpenGL contexts across the application to ensure VTK shaders and textures load properly.
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    QApplication a(argc, argv);
    QFile styleFile(":/styles/dark_blue.qss");
    if (styleFile.open(QFile::ReadOnly)) {
        QString styleSheet = QLatin1String(styleFile.readAll());
        a.setStyleSheet(styleSheet);
        styleFile.close();
    } else {
        qWarning() << "Failed to load stylesheet!";
    }

    std::shared_ptr<DataLoader> dl = std::make_shared<DataLoader>();
    std::shared_ptr<RegisterViewModel> regVM = std::make_shared<RegisterViewModel>(dl);
    MainWindow w(regVM);
    w.show();
    return a.exec();
}
