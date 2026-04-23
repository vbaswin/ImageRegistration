#pragma once
#include <QObject>

#include "dataloader.h"
#include "registration/DentalRegistrationEngine.h"
// #include "registrationmodel.h"
#include "registration/RegistrationPointEvaluator.h"

class RegisterViewModel : public QObject
{
    Q_OBJECT
public:
 explicit RegisterViewModel(std::shared_ptr<DataLoader> dl,
                            QObject* parent = nullptr);

 vtkSmartPointer<vtkPolyData> getStlData();
 vtkSmartPointer<vtkImageData> getDicomData();
 vtkSmartPointer<vtkPolyData> getSurfaceData(double contourValue);
 vtkSmartPointer<vtkPolyData> getRawSurfaceData(double contourValue);
 vtkSmartPointer<vtkVolumeProperty> getVolProps();
 vtkSmartPointer<vtkProperty> getSurfaceProps();
 void runDiagnosticCropTest();
 void savePoint(std::array<double, 3>, bool);
 void calculateRMS();
 void clearPoints();

 ImageRegistration::RegistrationResult performRegistration(double isoValue);
 void loadTestingDataset(int index);

signals:
 void dataLoaded();

private:
    std::shared_ptr<DataLoader> m_dataLoader;
    ImageRegistration::DentalRegistrationEngine m_registrationEngine;
    ImageRegistration::RegistrationPointEvaluator m_pointEvaluator;
};
