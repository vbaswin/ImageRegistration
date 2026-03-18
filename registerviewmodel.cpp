#include "registerviewmodel.h"

RegisterViewModel::RegisterViewModel(std::shared_ptr<DataLoader> dl, QObject *parent)
    : QObject(parent)
    , m_dataLoader(dl)
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
