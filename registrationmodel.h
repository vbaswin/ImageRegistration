#pragma once
#include <QDebug>
#include <QObject>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

class RegistrationModel : public QObject
{
    Q_OBJECT
public:
    explicit RegistrationModel(QObject *parent = nullptr);

    vtkSmartPointer<vtkMatrix4x4> computeTransform(vtkSmartPointer<vtkPolyData> sourceStl,
                                                   vtkSmartPointer<vtkPolyData> targetSurface);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertVtkToPcl(vtkSmartPointer<vtkPolyData> polyData);

signals:
};
