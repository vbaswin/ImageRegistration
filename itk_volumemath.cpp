#include "itk_volumemath.h"

#include <itkCastImageFilter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
// -----------------------------------------------------------------------------------------
// QUARANTINE ZONE: Only ITK and RegistrationBridge includes exist here.
// NO PCL HEADERS ALLOWED.
// -----------------------------------------------------------------------------------------
#include <itkEuler3DTransform.h>
#include <itkImageRegistrationMethodv4.h>
#include <itkMattesMutualInformationImageToImageMetricv4.h>
#include <itkRegularStepGradientDescentOptimizerv4.h>

#include <QDebug>

#include "RegistrationBridge.h"

vtkSmartPointer<vtkMatrix4x4> ITKVolumeMath::executeDistanceFieldRegistration(
    vtkSmartPointer<vtkPolyData> sourceStl,
    vtkSmartPointer<vtkImageData> targetCbct) {
    qDebug() << "[VOXEL REGISTRATION] Phase 0 - Coarse Auto-Centering...";
    // ----------------------------------------------------------------------------------
    // NEW ALGORITHM: Instantly snap the STL physically inside the CBCT volume
    // so the ITK Gradient Optimizer actually has overlapping anatomies to chew
    // on.
    // ----------------------------------------------------------------------------------
    double cbctBounds[6], stlBounds[6];
    targetCbct->GetBounds(cbctBounds);
    sourceStl->GetBounds(stlBounds);
    double cbctCenter[3] = {(cbctBounds[0] + cbctBounds[1]) / 2.0,
                            (cbctBounds[2] + cbctBounds[3]) / 2.0,
                            (cbctBounds[4] + cbctBounds[5]) / 2.0};
    double stlCenter[3] = {(stlBounds[0] + stlBounds[1]) / 2.0,
                           (stlBounds[2] + stlBounds[3]) / 2.0,
                           (stlBounds[4] + stlBounds[5]) / 2.0};
    // Calculate the physical translation gap
    vtkSmartPointer<vtkTransform> preTransform =
        vtkSmartPointer<vtkTransform>::New();
    preTransform->Translate(cbctCenter[0] - stlCenter[0],
                            cbctCenter[1] - stlCenter[1],
                            cbctCenter[2] - stlCenter[2]);
    // Shift the STL geometry into the CBCT immediately
    vtkSmartPointer<vtkTransformPolyDataFilter> autoCenterFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    autoCenterFilter->SetInputData(sourceStl);
    autoCenterFilter->SetTransform(preTransform);
    autoCenterFilter->Update();
    vtkSmartPointer<vtkPolyData> centeredStl = autoCenterFilter->GetOutput();
    // ----------------------------------------------------------------------------------
    qDebug()
        << "[VOXEL REGISTRATION] Phase 1 - Converting STL to Distance Field...";
    // Pass the perfectly CENTERED STL into your mathematically accelerated
    // Decimation Modeler
    auto vtkSdf =
        RegistrationBridge::GenerateSDFFromSTL(centeredStl, targetCbct);
    qDebug() << "[VOXEL REGISTRATION] Phase 2 - Zero-Copy ITK Memory Bridge...";
    DistanceFieldType::Pointer itkSDF;
    CBCTImageType::Pointer itkCBCT_short;

    try {
        itkSDF = RegistrationBridge::ConvertSDFToITK(vtkSdf);
        itkCBCT_short = RegistrationBridge::ConvertCBCTToITK(targetCbct);
    } catch (std::exception& e) {
        qWarning() << "Memory Bridge Failure. Aborting.";
        vtkSmartPointer<vtkMatrix4x4> identity =
            vtkSmartPointer<vtkMatrix4x4>::New();
        identity->Identity();
        return identity;
    }
    qDebug() << "[VOXEL REGISTRATION] Phase 2.5 - Casting CBCT to Float for "
                "Metrics...";
    using CastFilterType =
        itk::CastImageFilter<CBCTImageType, DistanceFieldType>;
    CastFilterType::Pointer caster = CastFilterType::New();
    caster->SetInput(itkCBCT_short);
    caster->Update();
    DistanceFieldType::Pointer itkCBCT_float = caster->GetOutput();
    qDebug() << "[VOXEL REGISTRATION] Phase 3 - Initializing Optimizer...";
    using TransformType = itk::Euler3DTransform<double>;
    using MetricType =
        itk::MattesMutualInformationImageToImageMetricv4<DistanceFieldType,
                                                         DistanceFieldType>;
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using RegistrationType =
        itk::ImageRegistrationMethodv4<DistanceFieldType, DistanceFieldType>;
    MetricType::Pointer metric = MetricType::New();
    metric->SetNumberOfHistogramBins(50);
    metric->UseMovingImageGradientFilterOff();
    OptimizerType::Pointer optimizer = OptimizerType::New();
    optimizer->SetLearningRate(4.0);
    optimizer->SetMinimumStepLength(0.01);
    optimizer->SetNumberOfIterations(250);
    TransformType::Pointer initialTransform = TransformType::New();
    initialTransform->SetIdentity();
    RegistrationType::Pointer registration = RegistrationType::New();

    registration->SetFixedImage(itkCBCT_float);
    registration->SetMovingImage(itkSDF);
    registration->SetMetric(metric);
    registration->SetOptimizer(optimizer);
    registration->SetInitialTransform(initialTransform);
    // Using Level 2 Multi-Resolution since we already Auto-Centered manually
    const unsigned int numberOfLevels = 2;
    RegistrationType::ShrinkFactorsArrayType shrinkFactors(numberOfLevels);
    shrinkFactors[0] = 4;
    shrinkFactors[1] = 1;
    RegistrationType::SmoothingSigmasArrayType smoothingSigmas(numberOfLevels);
    smoothingSigmas[0] = 2;
    smoothingSigmas[1] = 0;
    registration->SetNumberOfLevels(numberOfLevels);
    registration->SetShrinkFactorsPerLevel(shrinkFactors);
    registration->SetSmoothingSigmasPerLevel(smoothingSigmas);
    qDebug()
        << "[VOXEL REGISTRATION] Phase 4 - Executing Volume Mathematics...";
    try {
        registration->Update();
        qDebug() << "Optimization Score: " << optimizer->GetValue();
    } catch (itk::ExceptionObject& err) {
        qWarning() << "ITK Math Warning: " << err.GetDescription();
    }
    const TransformType* eulerTransform =
        dynamic_cast<const TransformType*>(registration->GetTransform());
    // Extract Phase 4 Fine ITK Matrix
    const TransformType::MatrixType& matrix = eulerTransform->GetMatrix();
    const TransformType::TranslationType& translation =
        eulerTransform->GetTranslation();
    vtkSmartPointer<vtkMatrix4x4> itkMatrix =
        vtkSmartPointer<vtkMatrix4x4>::New();
    itkMatrix->Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            itkMatrix->SetElement(r, c, matrix(r, c));
        }
        itkMatrix->SetElement(r, 3, translation[r]);
    }
    // ----------------------------------------------------------------------------------
    // PHASE 5: Matrix Concatenation
    // We MUST combine the Coarse VTK Auto-Center with the Fine ITK Optimizer.
    // Total Matrix = ITK_SubVoxel_Matrix * VTK_CenterOfMass_Matrix
    // ----------------------------------------------------------------------------------
    vtkSmartPointer<vtkMatrix4x4> finalCombinedMatrix =
        vtkSmartPointer<vtkMatrix4x4>::New();
    vtkMatrix4x4::Multiply4x4(itkMatrix, preTransform->GetMatrix(),
                              finalCombinedMatrix);
    return finalCombinedMatrix;
}
