#include "registerviewmodel.h"

RegisterViewModel::RegisterViewModel(std::shared_ptr<DataLoader> dl, QObject *parent)
    : QObject(parent)
    , m_dataLoader(dl)
{}

vtkSmartPointer<vtkPolyData> RegisterViewModel::getStlData()
{
    return m_dataLoader->getStlData();
}
