#include "DentalRegistrationEngine.h"

#include "../registrationmodel.h"
#include "VtkImageSurfaceExtractor.h"

namespace ImageRegistration {
namespace {

bool isEmpty(vtkPolyData* polyData) {
    return !polyData || polyData->GetNumberOfPoints() == 0;
}

bool isEmpty(vtkImageData* imageData) {
    return !imageData || imageData->GetNumberOfPoints() == 0;
}

}  // namespace

DentalRegistrationEngine::DentalRegistrationEngine()
    : m_registrationModel(std::make_unique<RegistrationModel>()) {}

DentalRegistrationEngine::~DentalRegistrationEngine() = default;

RegistrationResult DentalRegistrationEngine::registerStlToImage(
    const RegistrationInput& input, const RegistrationSettings& settings) {
    RegistrationResult result;
    result.transformMatrix->Identity();

    if (isEmpty(input.sourceStl)) {
        result.message = "Source STL is empty.";
        return result;
    }

    if (isEmpty(input.targetImage)) {
        result.message = "Target image data is empty.";
        return result;
    }

    vtkSmartPointer<vtkPolyData> targetEnamel;

    if (settings.imageAlreadyThresholded) {
        targetEnamel = VtkImageSurfaceExtractor::extractRawSurface(
            input.targetImage, settings.enamelIsoValue);
    } else {
        targetEnamel = VtkImageSurfaceExtractor::extractThresholdedSurface(
            input.targetImage, settings.thresholdMin, settings.thresholdMax,
            settings.enamelIsoValue);
    }

    vtkSmartPointer<vtkPolyData> targetEntireJaw =
        VtkImageSurfaceExtractor::extractRawSurface(input.targetImage,
                                                    settings.fullJawIsoValue);

    if (isEmpty(targetEnamel)) {
        result.message = "Could not extract enamel surface from target image.";
        return result;
    }

    if (isEmpty(targetEntireJaw)) {
        result.message =
            "Could not extract full jaw surface from target image.";
        return result;
    }

    m_registrationModel->computeTransform(input.sourceStl, targetEnamel,
                                          targetEntireJaw);

    vtkSmartPointer<vtkMatrix4x4> transform =
        m_registrationModel->getTransformMatrix();

    if (!transform) {
        result.message = "Registration did not produce a transform matrix.";
        return result;
    }

    result.transformMatrix->DeepCopy(transform.GetPointer());
    result.success = true;
    result.message = "Registration completed.";

    return result;
}

}  // namespace ImageRegistration
