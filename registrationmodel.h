#pragma once
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkCellData.h>
#include <vtkClipPolyData.h>
#include <vtkFloatArray.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>

#include <Eigen/Dense>
#include <QDebug>
#include <QObject>

class RegistrationModel : public QObject {
    Q_OBJECT
public:
    explicit RegistrationModel(QObject *parent = nullptr);

    vtkSmartPointer<vtkMatrix4x4> computeTransform(
        vtkSmartPointer<vtkPolyData> sourceStl,
        vtkSmartPointer<vtkPolyData> targetEnamel,
        vtkSmartPointer<vtkPolyData> targetEntireJaw);

    void saveDiagnosticCrop(vtkSmartPointer<vtkPolyData> inputStl,
                            const QString& outputPath);
    void saveDiagnosticPointCloud(const std::string& filename,
                                  const pcl::PointCloud<pcl::PointXYZ>& cloud);

   private:
    vtkSmartPointer<vtkPolyData> cropStlInVtk(
        vtkSmartPointer<vtkPolyData> inputMesh, float extractionThickness);

    pcl::PointCloud<pcl::PointXYZ>::Ptr applyMorphologicalClosing(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float closingRadius,
        float voxelResolution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractTeethRegion(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool, float);

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertVtkToPcl(
        vtkSmartPointer<vtkPolyData> polyData);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float leafSize);
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float searchRadius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals, float featureRadius);

    // Precision KD-Tree Spatial Extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractByProximityMask(
        pcl::PointCloud<pcl::PointXYZ>::Ptr maskCloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr targetSolidCloud,
        float searchRadius);

    vtkSmartPointer<vtkMatrix4x4> performRANSAC(
        pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals,
        pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures,
        float maxCorrespondenceDistance);

    vtkSmartPointer<vtkMatrix4x4> performICPWithNormals(
        pcl::PointCloud<pcl::PointNormal>::Ptr coarseSourceNormals,
        pcl::PointCloud<pcl::PointNormal>::Ptr coarseTargetNormals,
        pcl::PointCloud<pcl::PointNormal>::Ptr fineSourceNormals,
        pcl::PointCloud<pcl::PointNormal>::Ptr fineTargetNormals,
        vtkSmartPointer<vtkMatrix4x4> ransacTransform);

   signals:
};
