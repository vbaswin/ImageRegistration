#pragma once

#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <string>

namespace ImageRegistration {

struct RegistrationInput {
    vtkSmartPointer<vtkPolyData> sourceStl;
    vtkSmartPointer<vtkImageData> targetImage;
};

struct RegistrationSettings {
    double enamelIsoValue = 1800.0;
    double fullJawIsoValue = 400.0;

    double thresholdMin = 1800.0;
    double thresholdMax = 6000.0;

    bool imageAlreadyThresholded = false;
    bool enableDiagnostics = false;
    std::string diagnosticsDirectory;
};

struct RegistrationResult {
    bool success = false;
    std::string message;

    vtkSmartPointer<vtkMatrix4x4> transformMatrix =
        vtkSmartPointer<vtkMatrix4x4>::New();

    bool coarseIcpConverged = false;
    bool fineIcpConverged = false;
    double coarseFitnessScore = 0.0;
    double fineFitnessScore = 0.0;
};

}  // namespace ImageRegistration
