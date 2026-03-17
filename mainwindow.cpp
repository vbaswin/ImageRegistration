#include "mainwindow.h"
#include <QHBoxLayout>
#include <QPushButton>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUI();
    this->resize(1920, 1080);
}

MainWindow::~MainWindow() {}

void MainWindow::setupUI()
{
    m_container = new QWidget(this);
    this->setCentralWidget(m_container);
    QHBoxLayout *hLayout = new QHBoxLayout(m_container);

    m_vtkWidget = new QWidget(m_container);
    m_rightControlsWidget = new QWidget(m_container);
    m_rightControlsWidget->setObjectName("rightControls");

    hLayout->addWidget(m_vtkWidget, 6);
    hLayout->addWidget(m_rightControlsWidget, 1);

    QPushButton *autoRegisterBtn = new QPushButton("Auto Register", m_rightControlsWidget);
    autoRegisterBtn->setCheckable(true);
    // autoRegisterBtn->
    QVBoxLayout *vLayout = new QVBoxLayout(m_rightControlsWidget);
    vLayout->addWidget(autoRegisterBtn);
}
