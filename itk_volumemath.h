#pragma once
// -----------------------------------------------------------------------------------------
// ARCHITECTURE BARRIER: This header MUST remain 100% free of PCL and ITK
// includes. It bridges the Qt/PCL world to the isolated ITK world using
// standard VTK pointers.
// -----------------------------------------------------------------------------------------
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class ITKVolumeMath {
   public:
    static vtkSmartPointer<vtkMatrix4x4> executeDistanceFieldRegistration(
        vtkSmartPointer<vtkPolyData> sourceStl,
        vtkSmartPointer<vtkImageData> targetCbct);
};
