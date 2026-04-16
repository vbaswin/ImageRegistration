#pragma once
#include <QObject>
#include "dataloader.h"
#include "registrationmodel.h"

class RegisterViewModel : public QObject
{
    Q_OBJECT
public:
    explicit RegisterViewModel(std::shared_ptr<DataLoader> dl,
                               std::shared_ptr<RegistrationModel> regModel,
                               QObject *parent = nullptr);

    vtkSmartPointer<vtkPolyData> getStlData();
    vtkSmartPointer<vtkImageData> getDicomData();
    vtkSmartPointer<vtkPolyData> getSurfaceData(double contourValue);
    vtkSmartPointer<vtkPolyData> getRawSurfaceData(double contourValue);
    vtkSmartPointer<vtkVolumeProperty> getVolProps();
    vtkSmartPointer<vtkProperty> getSurfaceProps();
    void runDiagnosticCropTest();

    vtkSmartPointer<vtkMatrix4x4> performRegistration(double isoValue);
    void loadTestingDataset(int index);

   signals:
    void dataLoaded();

private:
    std::shared_ptr<DataLoader> m_dataLoader;
    std::shared_ptr<RegistrationModel> m_regModel;
};
