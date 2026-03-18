#pragma once
#include <QObject>
#include "dataloader.h"

class RegisterViewModel : public QObject
{
    Q_OBJECT
public:
    explicit RegisterViewModel(std::shared_ptr<DataLoader> dl, QObject *parent = nullptr);

    vtkSmartPointer<vtkPolyData> getStlData();
signals:
    void dataLoaded();

private:
    std::shared_ptr<DataLoader> m_dataLoader;
};

