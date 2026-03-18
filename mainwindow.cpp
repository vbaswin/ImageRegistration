#include "mainwindow.h"
#include <QHBoxLayout>
#include <QPushButton>

MainWindow::MainWindow(std::shared_ptr<RegisterViewModel> rVM, QWidget *parent)
    : QMainWindow(parent)
    , m_regVM(rVM)
{
    setupUI();
    setupData();
    this->resize(1920, 1080);
}

MainWindow::~MainWindow() {}

void MainWindow::setupUI()
{
    m_container = new QWidget(this);
    this->setCentralWidget(m_container);
    QHBoxLayout *hLayout = new QHBoxLayout(m_container);

    m_vtkWidget = new QVTKOpenGLNativeWidget(m_container);
    m_rightControlsWidget = new QWidget(m_container);
    m_rightControlsWidget->setObjectName("rightControls");

    hLayout->addWidget(m_vtkWidget, 6);
    hLayout->addWidget(m_rightControlsWidget, 1);

    QPushButton *autoRegisterBtn = new QPushButton("Auto Register", m_rightControlsWidget);
    autoRegisterBtn->setCheckable(true);
    // autoRegisterBtn->
    QVBoxLayout *vLayout = new QVBoxLayout(m_rightControlsWidget);
    vLayout->addWidget(autoRegisterBtn);

    m_vtkWidget->SetRenderWindow(m_renderWindow);

    m_renderWindow->AddRenderer(m_leftRenderer);
    m_renderWindow->AddRenderer(m_rightRenderer);

    m_leftRenderer->SetBackground(2 / 255.0, 8 / 255.0, 5 / 255.0);
    m_rightRenderer->SetBackground(7 / 255.0, 3 / 255.0, 6 / 255.0);

    // [xmin, ymin, xmax, ymax]
    double leftViewPort[4] = {0.0, 0.0, 0.5, 1.0};
    double rightViewPort[4] = {0.5, 0.0, 1.0, 1.0};

    m_leftRenderer->SetViewport(leftViewPort);
    m_rightRenderer->SetViewport(rightViewPort);

    // m_renderWindow->Render();
}

void MainWindow::setupData()
{
    m_stlPolyData = m_regVM->getStlData();
    m_stlMapper->SetInputData(m_stlPolyData);
    m_stlActor->SetMapper(m_stlMapper);
    m_leftRenderer->AddActor(m_stlActor);
}
