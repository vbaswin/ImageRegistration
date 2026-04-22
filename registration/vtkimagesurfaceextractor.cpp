#include "VtkImageSurfaceExtractor.h"

#include <vtkFlyingEdges3D.h>
#include <vtkImageThreshold.h>
#include <vtkNew.h>
#include <vtkPolyDataNormals.h>

namespace ImageRegistration {

vtkSmartPointer<vtkPolyData>
VtkImageSurfaceExtractor::extractThresholdedSurface(vtkImageData* imageData,
                                                    double thresholdMin,
                                                    double thresholdMax,
                                                    double isoValue) {
    vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();

    if (!imageData || imageData->GetNumberOfPoints() == 0) {
        return output;
    }

    vtkNew<vtkImageThreshold> thresholder;
    thresholder->SetInputData(imageData);
    thresholder->ThresholdBetween(thresholdMin, thresholdMax);
    thresholder->ReplaceOutOn();
    thresholder->SetOutValue(-1000);
    thresholder->Update();

    vtkNew<vtkFlyingEdges3D> isoAlgo;
    isoAlgo->SetInputConnection(thresholder->GetOutputPort());
    isoAlgo->SetValue(0, isoValue);
    isoAlgo->Update();

    vtkNew<vtkPolyDataNormals> normalsCalc;
    normalsCalc->SetInputConnection(isoAlgo->GetOutputPort());
    normalsCalc->SetFeatureAngle(60.0);
    normalsCalc->ComputePointNormalsOn();
    normalsCalc->ComputeCellNormalsOff();
    normalsCalc->ConsistencyOn();
    normalsCalc->AutoOrientNormalsOn();
    normalsCalc->SplittingOff();
    normalsCalc->Update();

    output->DeepCopy(normalsCalc->GetOutput());
    return output;
}

vtkSmartPointer<vtkPolyData> VtkImageSurfaceExtractor::extractRawSurface(
    vtkImageData* imageData, double isoValue) {
    vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();

    if (!imageData || imageData->GetNumberOfPoints() == 0) {
        return output;
    }

    vtkNew<vtkFlyingEdges3D> isoAlgo;
    isoAlgo->SetInputData(imageData);
    isoAlgo->SetValue(0, isoValue);
    isoAlgo->Update();

    vtkNew<vtkPolyDataNormals> normalsCalc;
    normalsCalc->SetInputConnection(isoAlgo->GetOutputPort());
    normalsCalc->ComputePointNormalsOn();
    normalsCalc->ConsistencyOn();
    normalsCalc->Update();

    output->DeepCopy(normalsCalc->GetOutput());
    return output;
}

}  // namespace ImageRegistration
