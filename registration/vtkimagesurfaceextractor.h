#pragma once

#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace ImageRegistration {

class VtkImageSurfaceExtractor {
   public:
    static vtkSmartPointer<vtkPolyData> extractThresholdedSurface(
        vtkImageData* imageData, double thresholdMin, double thresholdMax,
        double isoValue);

    static vtkSmartPointer<vtkPolyData> extractRawSurface(
        vtkImageData* imageData, double isoValue);
};

}  // namespace ImageRegistration
