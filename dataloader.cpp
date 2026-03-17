#include "dataloader.h"
#include "vtkSmartPointer.h"

DataLoader::DataLoader(QObject *parent)
    : QObject{parent}
{}

bool DataLoader::loadStl(QString &filePath)
{
    m_stlReader->SetFileName(filePath.toUtf8().constData());
    m_stlReader->Update();
    m_stlData = m_stlReader->GetOutput();
    if (m_stlData == nullptr || m_stlData->GetNumberOfPoints() == 0)
        return false;
    return true;
}
