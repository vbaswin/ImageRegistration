#pragma once
#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include "vtkImageData.h"

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
};
