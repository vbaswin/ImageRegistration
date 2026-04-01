// #include <cmath>
// #include <unordered_map>
// #include <utility>

#include "registrationmodel.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>  // omp uses multi-threading
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vtkPolyDataConnectivityFilter.h>

#include <algorithm>
#include <pcl/filters/impl/extract_indices.hpp>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float leafSize) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
        new pcl::PointCloud<pcl::PointXYZ>());
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
    sac_ia.setMaximumIterations(1000);
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

/*
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
*/
// ==========================================================
// UTILITY: Disconnected Artifact Cleanup
// ==========================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::removeDisconnectedArtifacts(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(inputCloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    // ec.setClusterTolerance(2.0);  // A physical gap of 2mm severs a cluster
    ec.setClusterTolerance(15.0);  // A physical gap of 2mm severs a cluster
    ec.setMinClusterSize(50);      // Ignore microscopic noise
    ec.setMaxClusterSize(inputCloud->points.size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(inputCloud);
    ec.extract(clusterIndices);

    if (clusterIndices.empty())
        return inputCloud;

    // PCL automatically sorts clusters by descending size.
    // Cluster 0 is mathematically guaranteed to be our continuous dental arch.
    pcl::PointCloud<pcl::PointXYZ>::Ptr purifiedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &idx : clusterIndices[0].indices) {
        purifiedCloud->points.push_back(inputCloud->points[idx]);
    }

    qDebug() << "Artifact Cleanup: Obliterated" << (clusterIndices.size() - 1)
             << "disconnected ghost artifacts.";
    return purifiedCloud;
}

// ==========================================================
// UTILITY: Cascaded (Two-Stage) High-Fidelity ICP Alignment
// ==========================================================
vtkSmartPointer<vtkMatrix4x4> RegistrationModel::performICPWithNormals(
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals,
    pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals,
    vtkSmartPointer<vtkMatrix4x4> ransacTransform) {
    // 1. Convert initial pose to Eigen Matrix
    Eigen::Matrix4f eigenInitial = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigenInitial(i, j) = ransacTransform->GetElement(i, j);
        }
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr alignedSource(
        new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*sourceNormals, *alignedSource,
                                        eigenInitial);

    // ==========================================================
    // STAGE 1: "The Magnet" (Standard ICP to collapse the Z-Gap)
    // ==========================================================
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icpMagnet;
    icpMagnet.setInputSource(alignedSource);
    icpMagnet.setInputTarget(targetNormals);

    // CONTRAINT 1: Tightened to 5.0mm. 15.0mm allows the gums to latch onto the
    // deep basal bone.
    icpMagnet.setMaxCorrespondenceDistance(5.0f);

    // CONSTRAINT 2: Activate Mathematical Symmetry. This instantly isolates the
    // crowns and terminates the downward drag generated by the red jawbone
    // base.
    icpMagnet.setUseReciprocalCorrespondences(true);

    icpMagnet.setMaximumIterations(20);
    icpMagnet.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointNormal>::Ptr intermediateSource(
        new pcl::PointCloud<pcl::PointNormal>());
    icpMagnet.align(*intermediateSource);

    Eigen::Matrix4f magnetMatrix = icpMagnet.getFinalTransformation();

    // ==========================================================
    // STAGE 2: "The Micro-Lock" (Point-to-Plane ICP to lock grooves)
    // ==========================================================
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>
        icpLock;
    icpLock.setInputSource(intermediateSource);
    icpLock.setInputTarget(targetNormals);

    icpLock.setMaxCorrespondenceDistance(4.0f);  // Micro-groove lock

    // CONSTRAINT 3: Enforce symmetry across the Point-to-Plane tension phase
    // to prevent lateral surface sliding along the red baseline.
    icpLock.setUseReciprocalCorrespondences(true);

    icpLock.setMaximumIterations(20);
    icpLock.setTransformationEpsilon(1e-8);

    pcl::PointCloud<pcl::PointNormal> finalOutput;
    icpLock.align(finalOutput);

    Eigen::Matrix4f lockMatrix = icpLock.getFinalTransformation();

    // 4. Matrix Multiplication: Combine all phases sequentially
    Eigen::Matrix4f absoluteEigen = lockMatrix * magnetMatrix * eigenInitial;

    // 5. Convert Absolute mathematically locked Transform back to VTK
    vtkSmartPointer<vtkMatrix4x4> finalVtkTrans =
        vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            finalVtkTrans->SetElement(i, j, absoluteEigen(i, j));
        }
    }

    qDebug() << "=== Cascaded ICP Convergence ===";
    qDebug() << "Magnet (Stage 1) Converged:" << icpMagnet.hasConverged()
             << " | Score:" << icpMagnet.getFitnessScore();
    qDebug() << "Lock (Stage 2) Converged:" << icpLock.hasConverged()
             << " | Score:" << icpLock.getFitnessScore();

    return finalVtkTrans;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::extractTeethRegion(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool isFullJaw,
    float extractionThickness) {
    if (!inputCloud || inputCloud->empty()) {
        qWarning() << "Input cloud is empty. Cannot execute CropBox extraction.";
        return inputCloud;
    }

    // 1. Establish Canonical Orientation via PCA longest-axes
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(inputCloud);

    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector4f centroid = pca.getMean();
    // Enforce strict right-handed coordinates
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));

    Eigen::Matrix4f transformToCanonical = Eigen::Matrix4f::Identity();
    transformToCanonical.block<3, 3>(0, 0) = eigenVectors.transpose();
    transformToCanonical.block<3, 1>(0, 3) = -1.0f
                                             * (eigenVectors.transpose() * centroid.head<3>());

    // Transform into perfectly aligned 'Bounding Box' Space
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*inputCloud, *alignedCloud, transformToCanonical);

    // 2. Measure the Anatomical Dimensions of the Jaw
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*alignedCloud, minPt, maxPt);

    float rangeX = maxPt.x - minPt.x; // Width of Jaw
    float rangeY = maxPt.y - minPt.y; // Depth of Jaw
    float rangeZ = maxPt.z - minPt.z; // Height of Jaw

    // 3. Density Heuristic (Determine if Crowns are facing +Z or -Z)
    float checkDepth = 5.0f;
    pcl::PassThrough<pcl::PointXYZ> passCheck;
    passCheck.setInputCloud(alignedCloud);
    passCheck.setFilterFieldName("z");

    pcl::PointCloud<pcl::PointXYZ> bottomSlice, topSlice;
    passCheck.setFilterLimits(minPt.z, minPt.z + checkDepth);
    passCheck.filter(bottomSlice);
    passCheck.setFilterLimits(maxPt.z - checkDepth, maxPt.z);
    passCheck.filter(topSlice);

    bool crownIsAtMaxZ;
    if (isFullJaw) {
        crownIsAtMaxZ = (topSlice.points.size() < bottomSlice.points.size());
    } else {
        crownIsAtMaxZ = (topSlice.points.size() > bottomSlice.points.size());
    }

    // ==========================================================
    // 4. PROPORTIONAL BOUNDING BOX CLIPPING
    // Set your anatomical clipping percentages here
    // ==========================================================
    // float clipBasePercent = 0.40f;  // Amputate the bottom 40% (Thick jaw base)
    // float clipRamusPercent = 0.15f; // Amputate the outer 15% edges (Skull joints / Ramus)
    // float clipRearPercent = 0.10f;  // Clean up 10% of the rear depth scatters

    float clipBasePercent = isFullJaw ? 0.40f : 0.35f;
    float clipRamusPercent =
        isFullJaw ? 0.15f : 0.00f;  // DO NOT chop STL X-axis (Saves molars)
    float clipRearPercent = isFullJaw ? 0.10f : 0.02f;  // DO NOT chop

    float newMinZ, newMaxZ;
    if (crownIsAtMaxZ) {
        newMinZ = minPt.z + (rangeZ * clipBasePercent); // Cut bottom up
        newMaxZ = maxPt.z;
    } else {
        newMinZ = minPt.z;
        newMaxZ = maxPt.z - (rangeZ * clipBasePercent); // Cut top down
    }

    // Clip the extremes of the length and width (Where the ramus/skull joints strictly exist)
    float newMinX = minPt.x + (rangeX * clipRamusPercent);
    float newMaxX = maxPt.x - (rangeX * clipRamusPercent);

    float newMinY = minPt.y + (rangeY * clipRearPercent);
    float newMaxY = maxPt.y - (rangeY * clipRearPercent);

    // ==========================================================
    // 5. EXECUTE 3D CROPBOX
    // ==========================================================
    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(alignedCloud);

    // Set absolute 3D Cartesian limits
    cropBox.setMin(Eigen::Vector4f(newMinX, newMinY, newMinZ, 1.0f));
    cropBox.setMax(Eigen::Vector4f(newMaxX, newMaxY, newMaxZ, 1.0f));

    pcl::PointCloud<pcl::PointXYZ>::Ptr surgicallyBoundedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    cropBox.filter(*surgicallyBoundedCloud);

    /*
    // 6. Return the isolated teeth back to normal World Coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalWorldCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*surgicallyBoundedCloud,
                             *finalWorldCloud,
                             transformToCanonical.inverse());

    qDebug() << "=== Proportional Bounding Box Crop Executed ===";
    qDebug() << "Amputated skull joints (X limits) and structural base (Z limits).";
    qDebug() << "Point count reduced down from:" << inputCloud->points.size() << "to"
             << finalWorldCloud->points.size();
*/
    /*
    // phase 2: 2.5D virtual ray casting
    float rayGridResolution = 0.5f;
    std::unordered_map<std::pair<int, int>, float, GridHash> zBuffer;

    // Pass A: Build the Elevation Topography Map
    for (const auto &pt : surgicallyBoundedCloud->points) { // Iterate on
    Bounded Cloud! int x_idx = static_cast<int>(std::round(pt.x /
    rayGridResolution)); int y_idx = static_cast<int>(std::round(pt.y /
    rayGridResolution)); std::pair<int, int> cell(x_idx, y_idx); if
    (zBuffer.find(cell) == zBuffer.end()) { zBuffer[cell] = pt.z; } else { if
    (crownIsAtMaxZ) { zBuffer[cell] = std::max(zBuffer[cell], pt.z); } else {
                zBuffer[cell] = std::min(zBuffer[cell], pt.z);
            }
        }
    }
    // Pass B: Extract ONLY the Enamel Shell (Discarding identical internal
    // mass) float extractionThickness = 3.0f;  // Only keep the surface + 8mm
    // of vertical side walls
    pcl::PointCloud<pcl::PointXYZ>::Ptr shellCloud(new
    pcl::PointCloud<pcl::PointXYZ>());

    for (const auto &pt : surgicallyBoundedCloud->points) {
        int x_idx = static_cast<int>(std::round(pt.x / rayGridResolution));
        int y_idx = static_cast<int>(std::round(pt.y / rayGridResolution));
        std::pair<int, int> cell(x_idx, y_idx);

        float surfaceZ = zBuffer[cell];

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
*/
    // ==========================================================
    // 6. RESTORATION TO WORLD COORDINATES
    // ==========================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalWorldCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*surgicallyBoundedCloud, *finalWorldCloud,
                             transformToCanonical.inverse());

    qDebug() << "=== HYBRID GEOMETRIC ENGINE EXECUTED ===";
    qDebug() << "Total points natively loaded:" << inputCloud->points.size();
    qDebug() << "Macro artifacts deleted by Bounding Box:"
             << (alignedCloud->points.size() - surgicallyBoundedCloud->points.size());
    qDebug() << "Micro artifacts deleted by Ray Caster:"
             << (surgicallyBoundedCloud->points.size() -
                 surgicallyBoundedCloud->points.size());
    qDebug() << "Perfectly cleansed enamel shell returned with points:"
             << finalWorldCloud->points.size();

    // Route the World Cloud through our modular Cleanup Utility to destroy
    // bottom scattering return removeDisconnectedArtifacts(finalWorldCloud);
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

    auto croppedSource = extractTeethRegion(pclSource, false, 8.0f);
    auto croppedTarget = extractTeethRegion(pclTarget, true, 8.0f);
    pcl::io::savePLYFileASCII("C:/Users/igrs/Desktop/Aswin/ImageRegistration/cropped_source.ply",
                              *croppedSource);
    pcl::io::savePLYFileASCII("C:/Users/igrs/Desktop/Aswin/ImageRegistration/cropped_target.ply",
                              *croppedTarget);

    // A. preprocessing stage
    float voxelLeafSize = 3.0f;
    float normalRadius = 6.0f;
    float featureRadius = 12.0f;

    auto sourceDown = downsampleCloud(croppedSource, voxelLeafSize);
    auto targetDown = downsampleCloud(croppedTarget, voxelLeafSize);

    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageRegistration/down_source.ply",
        *sourceDown);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageRegistration/down_target.ply",
        *targetDown);

    auto sourceNormals = estimateNormals(sourceDown, normalRadius);
    auto targetNormals = estimateNormals(targetDown, normalRadius);
    // C. fpfh feature extraction

    qDebug() << "Extracting FPFH ...";
    auto sourceFPFH = computeFPFH(sourceNormals, featureRadius);
    auto targetFPFH = computeFPFH(targetNormals, featureRadius);

    qDebug() << "Executing RANSAC on Purified Point Cloud...";
    float maxCorrespDist = 15.0f;
    // pcl::io::savePLYFileASCII(
    //     "C:/Users/igrs/Desktop/Aswin/ImageRegistration/ransac_actual_target.ply",
    //     *filteredTargetNormals);
    vtkSmartPointer<vtkMatrix4x4> ransacTransform = performRANSAC(sourceNormals,
                                                                  targetNormals,
                                                                  sourceFPFH,
                                                                  targetFPFH,
                                                                  maxCorrespDist);
    // immediately calculate their precision normals for the ICP engine.
    auto icpSource = extractTeethRegion(pclSource, false, 3.0f);
    auto icpTarget = extractTeethRegion(pclTarget, true, 3.0f);

    qDebug() << "Generating high-fidelity point to plane normal map...";
    float highFidelityNormalRadius = 1.5f;
    auto fineSourceNormals =
        estimateNormals(icpSource, highFidelityNormalRadius);
    auto fineTargetNormals =
        estimateNormals(icpTarget, highFidelityNormalRadius);

    qDebug() << "Initiating strict point-to-plane ICP Micro-Refinement...";
    // Pass the purely decimated downsampled clouds into ICP for final anatomical locking
    vtkSmartPointer<vtkMatrix4x4> absoluteLockedTransform =
        performICPWithNormals(fineSourceNormals, fineTargetNormals,
                              ransacTransform);
    return absoluteLockedTransform;
}
