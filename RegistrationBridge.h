#pragma once
// -----------------------------------------------------------------------------------------
// COMPONENT 1: The Distance Field Generator (VTK to Volumetric Math)
// This converts the optical surface scan (STL) into a dense 3D distance field
// so it can mathematically map to the CT volume gradients.
// -----------------------------------------------------------------------------------------
#include <vtkDecimatePro.h>
#include <vtkImageData.h>
#include <vtkImplicitModeller.h>
#include <vtkSmartPointer.h>

#include <algorithm>

// ITK headers for the zero-copy memory bridge
// Note: Requires ITKVtkGlue module to be enabled in CMake
#include <itkImage.h>
#include <itkVTKImageToImageFilter.h>

// Standard Types for Medical Data
using CBCTImageType =
    itk::Image<short, 3>;  // HU units are 16-bit signed shorts
using DistanceFieldType =
    itk::Image<float, 3>;  // Distances require float precision

class RegistrationBridge {
   public:
    // Step A: Convert the STL into an SDF Volume locked to the CBCT's physical
    // coordinate space
    static vtkSmartPointer<vtkImageData> GenerateSDFFromSTL(
        vtkSmartPointer<vtkPolyData> sourceStl,
        vtkSmartPointer<vtkImageData> targetCbct) {
        // 1. Extract the physical matrix geometry
        double cbctBounds[6];
        targetCbct->GetBounds(cbctBounds);

        // ==========================================================
        // SPEED OPTIMIZATION 1: Mesh Decimation
        // Reduces a 200k polygon scan to 4k polygons.
        // ==========================================================
        vtkSmartPointer<vtkDecimatePro> decimate =
            vtkSmartPointer<vtkDecimatePro>::New();
        decimate->SetInputData(sourceStl);
        decimate->SetTargetReduction(
            0.98);  // Delete 98% of high-frequency detail
        decimate->PreserveTopologyOn();
        decimate->Update();
        // ==========================================================
        // SPEED OPTIMIZATION 2: Diluted Grid Spacing
        // Forces the grid to evaluate every 1.5mm instead of every 0.2mm,
        // reducing voxels from 20 Million to ~200k while keeping aspect ratio.
        // ==========================================================
        double lengthX = cbctBounds[1] - cbctBounds[0];
        double lengthY = cbctBounds[3] - cbctBounds[2];
        double lengthZ = cbctBounds[5] - cbctBounds[4];

        int dimX = std::max(10, static_cast<int>(lengthX / 1.5));
        int dimY = std::max(10, static_cast<int>(lengthY / 1.5));
        int dimZ = std::max(10, static_cast<int>(lengthZ / 1.5));
        // 2. Configure the robust Distance Field mathematical modeler
        vtkSmartPointer<vtkImplicitModeller> implicitModeller =
            vtkSmartPointer<vtkImplicitModeller>::New();
        implicitModeller->SetInputConnection(decimate->GetOutputPort());
        implicitModeller->SetSampleDimensions(dimX, dimY, dimZ);
        implicitModeller->SetModelBounds(cbctBounds);
        // SPEED OPTIMIZATION 3: Cap math evaluation radius at 10mm
        implicitModeller->SetMaximumDistance(10.0);
        // Execute the conversion (This will now complete aggressively fast)
        implicitModeller->Update();
        return implicitModeller->GetOutput();
    }

    // Step B: The Zero-Copy bridge pushing VTK memory into the ITK Optimizer
    static CBCTImageType::Pointer ConvertCBCTToITK(
        vtkSmartPointer<vtkImageData> vtkCbct) {
        using VTKToITKFilterType = itk::VTKImageToImageFilter<CBCTImageType>;
        VTKToITKFilterType::Pointer filter = VTKToITKFilterType::New();
        filter->SetInput(vtkCbct);

        try {
            filter->Update();
        } catch (itk::ExceptionObject& e) {
            // Standard ITK fail-fast error trapping
            throw std::runtime_error(
                "ITK Matrix Conversion Failed: Ensure CBCT scalar type is "
                "SHORT.");
        }

        // Disconnect from pipeline to allow independent memory management
        CBCTImageType::Pointer itkImage = filter->GetOutput();
        itkImage->DisconnectPipeline();
        return itkImage;
    }

    static DistanceFieldType::Pointer ConvertSDFToITK(
        vtkSmartPointer<vtkImageData> vtkSdf) {
        using VTKToITKSdfFilterType =
            itk::VTKImageToImageFilter<DistanceFieldType>;
        VTKToITKSdfFilterType::Pointer filter = VTKToITKSdfFilterType::New();
        filter->SetInput(vtkSdf);
        filter->Update();

        DistanceFieldType::Pointer itkSdf = filter->GetOutput();
        itkSdf->DisconnectPipeline();
        return itkSdf;
    }
};
