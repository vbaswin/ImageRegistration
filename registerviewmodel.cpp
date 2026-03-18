#include "registerviewmodel.h"

RegisterViewModel::RegisterViewModel(std::shared_ptr<DataLoader> dl,
                                     std::shared_ptr<RegistrationModel> regModel,

                                     QObject *parent)
    : QObject(parent)
    , m_dataLoader(dl)
    , m_regModel(regModel)
{}

vtkSmartPointer<vtkPolyData> RegisterViewModel::getStlData()
{
    return m_dataLoader->getStlData();
}

vtkSmartPointer<vtkImageData> RegisterViewModel::getDicomData()
{
    return m_dataLoader->getDicomData();
}

vtkSmartPointer<vtkPolyData> RegisterViewModel::getSurfaceData(double contourValue)
{
    return m_dataLoader->getSurfaceData(contourValue);
}

vtkSmartPointer<vtkVolumeProperty> RegisterViewModel::getVolProps()
{
    return m_dataLoader->getVolProps();
}

vtkSmartPointer<vtkProperty> RegisterViewModel::getSurfaceProps()
{
    return m_dataLoader->getSurfaceProps();
}

vtkSmartPointer<vtkMatrix4x4> RegisterViewModel::performRegistration(double isoValue)
{
    vtkSmartPointer<vtkPolyData> sourceStl = getStlData();
    vtkSmartPointer<vtkPolyData> targetSurface = getSurfaceData(isoValue);

    return m_regModel->computeTransform(sourceStl, targetSurface);
}
