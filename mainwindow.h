#pragma once
#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkImageData.h"
#include "vtkRenderer.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setupUI();

private:
    QWidget *m_container = nullptr;
    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;
    QWidget *m_rightControlsWidget = nullptr;

    vtkNew<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkNew<vtkRenderer> m_leftRenderer;
    vtkNew<vtkRenderer> m_rightRenderer;
};
