#include "registrationmodel.h"

RegistrationModel::RegistrationModel(QObject *parent)
    : QObject{parent}
{}

vtkSmartPointer<vtkMatrix4x4> RegistrationModel::computeTransform(
    vtkSmartPointer<vtkPolyData> sourceStl, vtkSmartPointer<vtkPolyData> targetSurface)
{
    vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
    transform->Identity();
    return transform;
}
