// #include <cmath>
// #include <unordered_map>
// #include <utility>

#include "registrationmodel.h"
#include <algorithm>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/common.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h> // omp uses multi-threading
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include <vtkPolyDataConnectivityFilter.h>

RegistrationModel::RegistrationModel(QObject *parent)
    : QObject{parent}
{}

pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::convertVtkToPcl(
    vtkSmartPointer<vtkPolyData> polyData)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (polyData == nullptr || polyData->GetNumberOfPoints() == 0) {
        qDebug() << "PolyData is null";
        return pclCloud;
    }

    pcl::io::vtkPolyDataToPointCloud(polyData, *pclCloud);
    return pclCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::downsampleCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float leafSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxGrid;

    voxGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxGrid.setInputCloud(inputCloud);
    voxGrid.filter(*downsampled);

    return downsampled;
}

pcl::PointCloud<pcl::PointNormal>::Ptr RegistrationModel::estimateNormals(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float searchRadius)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud(new pcl::PointCloud<pcl::PointNormal>());

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normalEst;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    normalEst.setSearchMethod(tree);
    normalEst.setInputCloud(inputCloud);
    normalEst.setRadiusSearch(searchRadius);
    normalEst.compute(*normalCloud);

    //fpfh requires points and normals mapped together
    pcl::copyPointCloud(*inputCloud, *normalCloud);

    std::vector<int> dummyIndices;
    pcl::removeNaNFromPointCloud(*normalCloud, *normalCloud, dummyIndices);
    pcl::removeNaNNormalsFromPointCloud(*normalCloud, *normalCloud, dummyIndices);

    return normalCloud;
}
pcl::PointCloud<pcl::FPFHSignature33>::Ptr RegistrationModel::computeFPFH(
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals, float featureRadius)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeatures(
        new pcl::PointCloud<pcl::FPFHSignature33>());

    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>());

    fpfh.setSearchMethod(kdTree);
    fpfh.setInputCloud(cloudWithNormals);
    fpfh.setInputNormals(cloudWithNormals);
    fpfh.setRadiusSearch(featureRadius);
    fpfh.compute(*fpfhFeatures);

    return fpfhFeatures;
}

vtkSmartPointer<vtkMatrix4x4> RegistrationModel::performRANSAC(
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals,
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures,
    float maxCorrespondenceDistance)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>
        sac_ia;

    sac_ia.setInputSource(sourceNormals);
    sac_ia.setSourceFeatures(sourceFeatures);
    sac_ia.setInputTarget(targetNormals);
    sac_ia.setTargetFeatures(targetFeatures);
    sac_ia.setMaximumIterations(2000);
    sac_ia.setMaxCorrespondenceDistance(maxCorrespondenceDistance);

    pcl::PointCloud<pcl::PointNormal> alignedSource;
    sac_ia.align(alignedSource);

    // extract transformation matrix
    Eigen::Matrix4f initialTransform = sac_ia.getFinalTransformation();

    // convert eigen strictly back to vtk
    vtkSmartPointer<vtkMatrix4x4> vtkTrans = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vtkTrans->SetElement(i, j, initialTransform(i, j));
        }
    }

    return vtkTrans;
}
// Custom Hash to allow 2D grid mapping natively in C++ standard library
struct GridHash
{
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const
    {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::extractTeethRegion(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    if (!inputCloud || inputCloud->empty()) {
        qWarning() << "Input cloud is empty. Cannot extract teeth crowns.";
        return inputCloud;
    }

    // 1. Establish the Spatial Canonical Orientation via PCA longest-axes
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(inputCloud);

    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector4f centroid = pca.getMean();
    // Enforce strict right-handed coordinates to prevent spatial mirroring
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));

    Eigen::Matrix4f transformToCanonical = Eigen::Matrix4f::Identity();
    transformToCanonical.block<3, 3>(0, 0) = eigenVectors.transpose();
    transformToCanonical.block<3, 1>(0, 3) = -1.0f
                                             * (eigenVectors.transpose() * centroid.head<3>());

    // Transform the full CBCT into the 'overhead' Canonical Space
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*inputCloud, *alignedCloud, transformToCanonical);

    // 2. Discover Biological Orientation (Calculate if crowns point Upper or Lower Z)
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*alignedCloud, minPt, maxPt);

    float checkDepth = 5.0f;
    pcl::PassThrough<pcl::PointXYZ> passCheck;
    passCheck.setInputCloud(alignedCloud);
    passCheck.setFilterFieldName("z");

    pcl::PointCloud<pcl::PointXYZ> bottomSlice, topSlice;
    passCheck.setFilterLimits(minPt.z, minPt.z + checkDepth);
    passCheck.filter(bottomSlice);
    passCheck.setFilterLimits(maxPt.z - checkDepth, maxPt.z);
    passCheck.filter(topSlice);

    bool crownIsAtMaxZ = (topSlice.points.size() < bottomSlice.points.size());

    // ==========================================================
    // 3. VIRTUAL RAY CASTING (2.5D ELEVATION Z-BUFFER MAP)
    // ==========================================================
    float rayGridResolution = 0.5f; // Shoot a ray constraint every 0.5mm

    // Utilizing your previously authored GridHash
    std::unordered_map<std::pair<int, int>, float, GridHash> zBuffer;

    // Pass A: Ray collision detection (Find the highest/lowest exterior point for every grid sector)
    for (const auto &pt : alignedCloud->points) {
        int x_idx = static_cast<int>(std::round(pt.x / rayGridResolution));
        int y_idx = static_cast<int>(std::round(pt.y / rayGridResolution));
        std::pair<int, int> cell(x_idx, y_idx);

        if (zBuffer.find(cell) == zBuffer.end()) {
            zBuffer[cell] = pt.z;
        } else {
            if (crownIsAtMaxZ) {
                zBuffer[cell] = std::max(zBuffer[cell], pt.z);
            } else {
                zBuffer[cell] = std::min(zBuffer[cell], pt.z);
            }
        }
    }

    // Pass B: Volumetric Extraction (Extract the hit-surface + a highly specific enamel thickness)
    float extractionThickness = 8.0f; // Slice precisely 8mm beneath the ray collision map
    pcl::PointCloud<pcl::PointXYZ>::Ptr shellCloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto &pt : alignedCloud->points) {
        int x_idx = static_cast<int>(std::round(pt.x / rayGridResolution));
        int y_idx = static_cast<int>(std::round(pt.y / rayGridResolution));
        std::pair<int, int> cell(x_idx, y_idx);

        float surfaceZ = zBuffer[cell];

        // Keep ONLY the structural enamel shell points that are within 'extractionThickness' of the collision
        if (crownIsAtMaxZ) {
            if (pt.z >= surfaceZ - extractionThickness) {
                shellCloud->points.push_back(pt);
            }
        } else {
            if (pt.z <= surfaceZ + extractionThickness) {
                shellCloud->points.push_back(pt);
            }
        }
    }

    // 4. Safely Transport the clean Ray-Casted shell back to its Original World Coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalWorldCloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f transformToOriginal = transformToCanonical.inverse();
    pcl::transformPointCloud(*shellCloud, *finalWorldCloud, transformToOriginal);

    // Telemetry output for debugging physical dimensions
    qDebug() << "=== 2.5D Ray Cast Extraction Executed ===";
    qDebug() << "Total Z-Buffer rays mapped correctly across top surface:" << zBuffer.size();
    qDebug() << "Ray Cast Obliterated internal bone, points radically reduced from:"
             << inputCloud->points.size() << "to" << finalWorldCloud->points.size();

    return finalWorldCloud;
}

vtkSmartPointer<vtkMatrix4x4> RegistrationModel::computeTransform(
    vtkSmartPointer<vtkPolyData> sourceStl, vtkSmartPointer<vtkPolyData> targetSurface)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSource = convertVtkToPcl(sourceStl);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTarget = convertVtkToPcl(targetSurface);

    pcl::io::savePLYFileASCII("C:/Users/igrs/Desktop/Aswin/ImageRegistration/initial_target.ply",
                              *pclTarget);
    pcl::io::savePLYFileASCII("C:/Users/igrs/Desktop/Aswin/ImageRegistration/initial_source.ply",
                              *pclSource);

    vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
    transform->Identity();

    if (pclSource->points.empty() || pclTarget->points.empty()) {
        vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
        transform->Identity();
        return transform;
    }

    auto croppedTarget = extractTeethRegion(pclTarget);
    pcl::io::savePLYFileASCII("C:/Users/igrs/Desktop/Aswin/ImageRegistration/cropped_target.ply",
                              *croppedTarget);

    // A. preprocessing stage
    float voxelLeafSize = 1.0f;
    float normalRadius = 2.0f;

    auto sourceDown = downsampleCloud(pclSource, voxelLeafSize);
    auto targetDown = downsampleCloud(croppedTarget, voxelLeafSize);

    auto sourceNormals = estimateNormals(sourceDown, normalRadius);
    auto targetNormals = estimateNormals(targetDown, normalRadius);
    // auto sourceNormals = estimateNormals(pclSource, normalRadius);
    // auto targetNormals = estimateNormals(pclTarget, normalRadius);

    // B. orientation invariant filtering - curvature extraction

    // helper lambda to extract curvature points
    // between .02 and 0.08, if too much bone increase
    // if teeth disappear decrease it
    // float targetCurvatureThreshold = 0.1f;

    // on both cbct and stl
    // auto featuredSource = extractHighCurvature(sourceNormals, targetCurvatureThreshold);
    // auto featuredTarget = extractHighCurvature(targetNormals, targetCurvatureThreshold);

    // pcl::io::savePLYFileASCII("C:/Users/igrs/Desktop/Aswin/ImageRegistration/debug_dicom_teeth.ply",
    //                           *featuredTarget);

    // C. fpfh feature extraction
    float featureRadius = 10.0f;

    qDebug() << "Extracting FPFH ...";
    auto sourceFPFH = computeFPFH(sourceNormals, featureRadius);
    auto targetFPFH = computeFPFH(targetNormals, featureRadius);

    qDebug() << "Executing RANSAC on Purified Point Cloud...";
    float maxCorrespDist = 15.0f;
    // pcl::io::savePLYFileASCII(
    //     "C:/Users/igrs/Desktop/Aswin/ImageRegistration/ransac_actual_target.ply",
    //     *filteredTargetNormals);
    // Notice we feed it the Highly Filtered CBCT, starving out the RANSAC outliers
    vtkSmartPointer<vtkMatrix4x4> ransacTransform = performRANSAC(sourceNormals,
                                                                  targetNormals,
                                                                  sourceFPFH,
                                                                  targetFPFH,
                                                                  maxCorrespDist);
    qDebug() << "RANSAC Execution complete!";
    return ransacTransform;
}
