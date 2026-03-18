#pragma once
#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include "registerviewmodel.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkImageData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderer.h"
#include <memory>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(std::shared_ptr<RegisterViewModel> rVm, QWidget *parent = nullptr);
    ~MainWindow();

    void setupUI();
    void setupData();

private:
    QWidget *m_container = nullptr;
    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;
    QWidget *m_rightControlsWidget = nullptr;

    vtkNew<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkNew<vtkRenderer> m_leftRenderer;
    vtkNew<vtkRenderer> m_rightRenderer;
    vtkNew<vtkPolyDataMapper> m_stlMapper;
    vtkNew<vtkActor> m_stlActor;
    // vtkNew<vtkRenderer> m_rightRenderer;
    vtkSmartPointer<vtkPolyData> m_stlPolyData;
    std::shared_ptr<RegisterViewModel> m_regVM;
};
