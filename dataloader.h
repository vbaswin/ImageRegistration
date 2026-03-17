#pragma once
#include <QObject>
#include "vtkPolyData.h"
#include "vtkSTLReader.h"

class DataLoader : public QObject
{
    Q_OBJECT
public:
    explicit DataLoader(QObject *parent = nullptr);
    bool loadStl(QString &filePath);
    // bool loadCbct(QString &filePath);

signals:
    void stlLoaded();
    void cbctLoaded();

private:
    vtkSmartPointer<vtkPolyData> m_stlData;
    // vtkNew<vtkPolyData> m_cbctData;
    vtkSmartPointer<vtkSTLReader> m_stlReader;
    // vtkNew<vtkSTLReader> m_stlReader;
};
