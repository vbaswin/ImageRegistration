// #include <cmath>
// #include <unordered_map>
// #include <utility>

#include "registrationmodel.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>  // omp uses multi-threading
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
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
pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::extractTeethRegion(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool isFullJaw,
    float extractionThickness) {
    if (!inputCloud || inputCloud->empty()) {
        qWarning() << "Input cloud empty.";
        return inputCloud;
    }

    // ==========================================================
    // PHASE 1: The Smoothing Proxy Engine (Solve DICOM Jaggedness)
    // ==========================================================
    // 1A: Aggressively merge microscopic voxel steps
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(1.5f, 1.5f, 1.5f);  // Erases standard DICOM staircases
    grid.setInputCloud(inputCloud);
    grid.filter(*downsampledCloud);

    // 1B: Moving Least Squares (MLS) - Mathematically sand the bone smooth
    pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothProxy(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false);
    mls.setInputCloud(downsampledCloud);
    mls.setPolynomialOrder(1);  // Linear fit perfectly smooths jaggedness
    mls.setSearchMethod(mls_tree);
    mls.setSearchRadius(4.0);  // Blends physics across 4mm overlapping patches
    mls.process(*smoothProxy);

    // ==========================================================
    // PHASE 2: Anti-Ramus Quarantine Zone
    // ==========================================================
    Eigen::Vector4f global_centroid;
    pcl::compute3DCentroid(*smoothProxy, global_centroid);

    pcl::PointCloud<pcl::PointXYZ>::Ptr coreCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    float coreRadiusSq = 45.0f * 45.0f;  // Excludes the posterior ramus

    for (const auto& pt : smoothProxy->points) {
        float distSq =
            (pt.x - global_centroid.x()) * (pt.x - global_centroid.x()) +
            (pt.y - global_centroid.y()) * (pt.y - global_centroid.y()) +
            (pt.z - global_centroid.z()) * (pt.z - global_centroid.z());

        if (distSq < coreRadiusSq) {
            coreCloud->points.push_back(pt);
        }
    }
    if (coreCloud->points.size() < 100) {
        coreCloud = smoothProxy;
    }

    // ==========================================================
    // PHASE 3: Curvature Measurement on the Healed Surface
    // ==========================================================
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(coreCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(4.0);

    pcl::PointCloud<pcl::PointNormal>::Ptr normals(
        new pcl::PointCloud<pcl::PointNormal>());
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roughCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < coreCloud->points.size(); ++i) {
        // Because the bone is now mathematically "glass", this threshold
        // aggressively identifies ONLY true, sharp teeth cusps.
        if (normals->points[i].curvature > 0.04) {
            roughCloud->points.push_back(coreCloud->points[i]);
        }
    }
    if (roughCloud->points.size() < 100) {
        roughCloud = coreCloud;
    }

    // ==========================================================
    // PHASE 4: RANSAC Occlusal Plane Discovery
    // ==========================================================
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(3.0);
    seg.setMaxIterations(1000);
    seg.setInputCloud(roughCloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        qWarning() << "Occlusal Plane calculation failed.";
        return inputCloud;
    }

    // ==========================================================
    // PHASE 5: Global Tilt Eradication (Applied to Original Cloud)
    // ==========================================================
    Eigen::Vector3f plane_normal(coefficients->values[0],
                                 coefficients->values[1],
                                 coefficients->values[2]);
    plane_normal.normalize();

    Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
    float cos_theta = plane_normal.dot(z_axis);
    Eigen::Vector3f rotation_axis = plane_normal.cross(z_axis).normalized();
    float angle = std::acos(cos_theta);

    Eigen::Matrix4f transform_to_canonical = Eigen::Matrix4f::Identity();
    if (std::abs(cos_theta) < 0.999f) {
        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(angle, rotation_axis);
        transform_to_canonical.block<3, 3>(0, 0) = rotation;
    }

    Eigen::Vector4f plane_centroid;
    pcl::compute3DCentroid(*roughCloud, *inliers, plane_centroid);
    transform_to_canonical.block<3, 1>(0, 3) =
        -1.0f *
        (transform_to_canonical.block<3, 3>(0, 0) * plane_centroid.head<3>());

    // VERY IMPORTANT: Apply the calculated matrix to the unsmoothed inputCloud!
    pcl::PointCloud<pcl::PointXYZ>::Ptr leveledCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*inputCloud, *leveledCloud,
                             transform_to_canonical);

    // ==========================================================
    // PHASE 6: Gravity Polarity Enforcement
    // ==========================================================
    Eigen::Vector4f full_centroid;
    pcl::compute3DCentroid(*leveledCloud, full_centroid);

    if (full_centroid.z() > 0) {
        Eigen::Matrix4f flip = Eigen::Matrix4f::Identity();
        flip(1, 1) = -1.0f;
        flip(2, 2) = -1.0f;
        transform_to_canonical = flip * transform_to_canonical;
        pcl::transformPointCloud(*inputCloud, *leveledCloud,
                                 transform_to_canonical);
    }

    // ==========================================================
    // PHASE 7: Pristine Anatomical Crown Extractor
    // ==========================================================
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*leveledCloud, minPt, maxPt);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(leveledCloud);
    pass.setFilterFieldName("z");

    float safeCrownHeight =
        extractionThickness > 0.1f ? extractionThickness : 12.0f;
    pass.setFilterLimits(maxPt.z - safeCrownHeight, maxPt.z + 1.0f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr leveledCrowns(
        new pcl::PointCloud<pcl::PointXYZ>());
    pass.filter(*leveledCrowns);

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalWorldCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*leveledCrowns, *finalWorldCloud,
                             transform_to_canonical.inverse());

    return finalWorldCloud;
}

// ==========================================================
// UTILITY: Disconnected Artifact Cleanup
// ==========================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr
RegistrationModel::removeDisconnectedArtifacts(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
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

    if (clusterIndices.empty()) return inputCloud;

    // PCL automatically sorts clusters by descending size.
    // Cluster 0 is mathematically guaranteed to be our continuous dental arch.
    pcl::PointCloud<pcl::PointXYZ>::Ptr purifiedCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& idx : clusterIndices[0].indices) {
        purifiedCloud->points.push_back(inputCloud->points[idx]);
    }

    qDebug() << "Artifact Cleanup: Obliterated" << (clusterIndices.size() - 1)
             << "disconnected ghost artifacts.";
    return purifiedCloud;
}

// ==========================================================
// UTILITY: Cascaded (Two-Stage Multi-Resolution) High-Fidelity ICP
// ==========================================================
vtkSmartPointer<vtkMatrix4x4> RegistrationModel::performICPWithNormals(
    pcl::PointCloud<pcl::PointNormal>::Ptr coarseSourceNormals,
    pcl::PointCloud<pcl::PointNormal>::Ptr coarseTargetNormals,
    pcl::PointCloud<pcl::PointNormal>::Ptr fineSourceNormals,
    pcl::PointCloud<pcl::PointNormal>::Ptr fineTargetNormals,
    vtkSmartPointer<vtkMatrix4x4> ransacTransform) {
    // 1. Convert initial pose to Eigen Matrix
    Eigen::Matrix4f eigenInitial = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigenInitial(i, j) = ransacTransform->GetElement(i, j);
        }
    }

    // 2. Apply initial RANSAC orientation to BOTH the coarse and fine source
    // clouds
    pcl::PointCloud<pcl::PointNormal>::Ptr alignedCoarseSource(
        new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*coarseSourceNormals,
                                        *alignedCoarseSource, eigenInitial);

    pcl::PointCloud<pcl::PointNormal>::Ptr alignedFineSource(
        new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*fineSourceNormals, *alignedFineSource,
                                        eigenInitial);

    // ==========================================================
    // STAGE 1: "The Magnet" (Executes FAST on Coarse points)
    // ==========================================================
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icpMagnet;
    icpMagnet.setInputSource(alignedCoarseSource);
    icpMagnet.setInputTarget(coarseTargetNormals);

    // CONSTRAINT 1: Allow 5.0mm spatial reach to pull the models together.
    icpMagnet.setMaxCorrespondenceDistance(5.0f);
    // CONSTRAINT 2: Symmetry drastically prevents sliding.
    icpMagnet.setUseReciprocalCorrespondences(true);
    icpMagnet.setMaximumIterations(20);
    icpMagnet.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointNormal>::Ptr intermediateCoarseSource(
        new pcl::PointCloud<pcl::PointNormal>());
    icpMagnet.align(*intermediateCoarseSource);

    Eigen::Matrix4f magnetMatrix = icpMagnet.getFinalTransformation();

    // 3. Mathematically advance the pure HIGH-RESOLUTION cloud using the Stage
    // 1 Matrix
    pcl::PointCloud<pcl::PointNormal>::Ptr intermediateFineSource(
        new pcl::PointCloud<pcl::PointNormal>());
    pcl::transformPointCloudWithNormals(*alignedFineSource,
                                        *intermediateFineSource, magnetMatrix);

    // ==========================================================
    // STAGE 2: "The Micro-Lock" (Executes precise lock on Fine points)
    // ==========================================================
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>
        icpLock;
    icpLock.setInputSource(intermediateFineSource);
    icpLock.setInputTarget(fineTargetNormals);

    // STRICT CONSTRAINT: Cut correspondence distance right down for
    // micro-grooves.
    icpLock.setMaxCorrespondenceDistance(2.0f);
    icpLock.setUseReciprocalCorrespondences(true);

    // ARCHITECTURAL SPEED FIX: The Magnet has already done the heavy lifting.
    // Do NOT let the dense cloud calculate 20 iterations. Clamp at 5.
    icpLock.setMaximumIterations(5);
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

    qDebug() << "=== Cascaded Multi-Resolution ICP Convergence ===";
    qDebug() << "Coarse Magnet (Stage 1) Converged:" << icpMagnet.hasConverged()
             << " | Score:" << icpMagnet.getFitnessScore();
    qDebug() << "Fine Lock (Stage 2) Converged:" << icpLock.hasConverged()
             << " | Score:" << icpLock.getFitnessScore();

    return finalVtkTrans;
}

vtkSmartPointer<vtkMatrix4x4> RegistrationModel::computeTransform(
    vtkSmartPointer<vtkPolyData> sourceStl,
    vtkSmartPointer<vtkPolyData> targetSurface) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSource = convertVtkToPcl(sourceStl);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTarget = convertVtkToPcl(targetSurface);

    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/initial_target.ply",
        *pclTarget);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/initial_source.ply",
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
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/cropped_source.ply",
        *croppedSource);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/cropped_target.ply",
        *croppedTarget);

    // A. preprocessing stage
    float voxelLeafSize = 3.0f;
    float normalRadius = 6.0f;
    float featureRadius = 12.0f;

    auto sourceDown = downsampleCloud(croppedSource, voxelLeafSize);
    auto targetDown = downsampleCloud(croppedTarget, voxelLeafSize);

    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/down_source.ply",
        *sourceDown);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/down_target.ply",
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

    // 2. Generate the DECIMATED Macro-Map (Used for High-Speed Magnet
    // alignment)
    qDebug()
        << "Generating decimated normals for high-speed macro alignment...";
    float coarseLeafSize = 1.0f;  // Eliminates massive calculation overhead
    auto coarseSource = downsampleCloud(icpSource, coarseLeafSize);
    auto coarseTarget = downsampleCloud(icpTarget, coarseLeafSize);
    auto coarseSourceNormals =
        estimateNormals(coarseSource, 2.0f);  // Moderate normal search
    auto coarseTargetNormals = estimateNormals(coarseTarget, 2.0f);
    qDebug() << "Initiating Multi-Resolution Cascaded ICP...";
    vtkSmartPointer<vtkMatrix4x4> absoluteLockedTransform =
        performICPWithNormals(coarseSourceNormals, coarseTargetNormals,
                              fineSourceNormals, fineTargetNormals,
                              ransacTransform);
    return absoluteLockedTransform;
}
