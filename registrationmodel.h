#pragma once
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <QDebug>
#include <QObject>

class RegistrationModel : public QObject {
    Q_OBJECT
public:
    explicit RegistrationModel(QObject *parent = nullptr);

    vtkSmartPointer<vtkMatrix4x4> computeTransform(
        vtkSmartPointer<vtkPolyData> sourceStl,
        vtkSmartPointer<vtkPolyData> targetSurface);

   private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractTeethRegion(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertVtkToPcl(vtkSmartPointer<vtkPolyData> polyData);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float leafSize);
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float searchRadius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals, float featureRadius);

    vtkSmartPointer<vtkMatrix4x4> performRANSAC(
        pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals,
        pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures,
        float maxCorrespondenceDistance);

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeDisconnectedArtifacts(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    vtkSmartPointer<vtkMatrix4x4> performICPWithNormals(
        pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals,
        pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals,
        vtkSmartPointer<vtkMatrix4x4> ransacTransform);

   signals:
};
