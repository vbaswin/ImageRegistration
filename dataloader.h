#pragma once
#include <QObject>
#include "vtkSTLReader.h"
#include <vtkPolyData.h>

class DataLoader : public QObject
{
    Q_OBJECT
public:
    explicit DataLoader(QObject *parent = nullptr);
    bool loadStl(QString &filePath);
    // bool loadCbct(QString &filePath);
    vtkSmartPointer<vtkPolyData> getStlData();

signals:
    void stlLoaded();
    void cbctLoaded();

private:
    vtkSmartPointer<vtkPolyData> m_stlData;
    // vtkNew<vtkPolyData> m_cbctData;
    vtkNew<vtkSTLReader> m_stlReader;
};
