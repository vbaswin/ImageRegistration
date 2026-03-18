#pragma once
#include <QObject>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class RegistrationModel : public QObject
{
    Q_OBJECT
public:
    explicit RegistrationModel(QObject *parent = nullptr);

    vtkSmartPointer<vtkMatrix4x4> computeTransform(vtkSmartPointer<vtkPolyData> sourceStl,
                                                   vtkSmartPointer<vtkPolyData> targetSurface);

signals:
};
