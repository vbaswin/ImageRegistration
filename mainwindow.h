#pragma once
#include <QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setupUI();

private:
    QWidget *m_container = nullptr;
    QWidget *m_vtkWidget = nullptr;
    QWidget *m_rightControlsWidget = nullptr;
};
