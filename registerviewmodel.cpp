#include "registerviewmodel.h"

RegisterViewModel::RegisterViewModel(std::shared_ptr<DataLoader> dl,
                                     QObject* parent)
    : QObject(parent), m_dataLoader(dl) {}

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

vtkSmartPointer<vtkPolyData> RegisterViewModel::getRawSurfaceData(
    double contourValue) {
    return m_dataLoader->getRawSurfaceData(contourValue);
}

vtkSmartPointer<vtkVolumeProperty> RegisterViewModel::getVolProps()
{
    return m_dataLoader->getVolProps();
}

vtkSmartPointer<vtkProperty> RegisterViewModel::getSurfaceProps()
{
    return m_dataLoader->getSurfaceProps();
}
ImageRegistration::RegistrationResult RegisterViewModel::performRegistration(
    double isoValue) {
    ImageRegistration::RegistrationInput input;
    input.sourceStl = getStlData();
    input.targetImage = m_dataLoader->getDicomDataNoFilter();

    ImageRegistration::RegistrationSettings settings;
    settings.enamelIsoValue = isoValue;
    settings.fullJawIsoValue = 400.0;
    settings.thresholdMin = 1800.0;
    settings.thresholdMax = 6000.0;
    settings.imageAlreadyThresholded = false;
    settings.enableDiagnostics = false;

    ImageRegistration::RegistrationResult result =
        m_registrationEngine.registerStlToImage(input, settings);

    m_pointEvaluator.setTransform(
        result.success ? result.transformMatrix.GetPointer() : nullptr);

    return result;
}

void RegisterViewModel::loadTestingDataset(int index) {
    m_dataLoader->loadTestingDataset(index);
}

void RegisterViewModel::runDiagnosticCropTest() {
    vtkSmartPointer<vtkPolyData> sourceStl = getStlData();

    // We establish the isolated test path here in the ViewModel abstraction
    // layer
    QString debugPath =
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/vtk_cropped.stl";

    // Safely asks the Engine layer to perform its job and dump it to the debug
    // folder
    // m_regModel->saveDiagnosticCrop(sourceStl, debugPath);
}
void RegisterViewModel::clearPoints() { m_pointEvaluator.clearPoints(); }

void RegisterViewModel::calculateRMS() { m_pointEvaluator.calculateRMS(); }

void RegisterViewModel::savePoint(std::array<double, 3> newPoint, bool isStl) {
    m_pointEvaluator.savePoint(newPoint, isStl);
}
