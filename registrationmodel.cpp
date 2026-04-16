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
#include <vtkFeatureEdges.h>
#include <vtkPolyDataConnectivityFilter.h>

#include <algorithm>
#include <cstdint>
#include <pcl/filters/impl/extract_indices.hpp>
#include <queue>
#include <unordered_set>
#include <vector>

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

pcl::PointCloud<pcl::PointXYZ>::Ptr
RegistrationModel::applyMorphologicalClosing(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, float closingRadius,
    float voxelResolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (!inputCloud || inputCloud->empty()) return outputCloud;

    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);

    int pad = static_cast<int>(std::ceil(closingRadius / voxelResolution)) + 2;
    int w = static_cast<int>((maxPt.x() - minPt.x()) / voxelResolution) +
            (2 * pad) + 1;
    int h = static_cast<int>((maxPt.y() - minPt.y()) / voxelResolution) +
            (2 * pad) + 1;
    int d = static_cast<int>((maxPt.z() - minPt.z()) / voxelResolution) +
            (2 * pad) + 1;

    if (static_cast<size_t>(w) * h * d > 100000000) {
        qWarning() << "Voxel Grid resolution too tight! Aborting morphological "
                      "operations.";
        return inputCloud;
    }

    // [MSVC COMPILER FIX]: Using strictly prefixed constexpr memory instead of
    // unscoped enums
    const uint8_t VOX_EMPTY = 0;
    const uint8_t VOX_OCCUPIED = 1;
    const uint8_t VOX_DILATED = 2;
    const uint8_t VOX_EXTERIOR = 3;

    std::vector<uint8_t> grid(w * h * d, VOX_EMPTY);

    auto idx = [&](int x, int y, int z) -> size_t {
        return static_cast<size_t>(z * w * h + y * w + x);
    };

    auto posToGrid = [&](const pcl::PointXYZ& pt) -> Eigen::Vector3i {
        return Eigen::Vector3i(
            static_cast<int>((pt.x - minPt.x()) / voxelResolution) + pad,
            static_cast<int>((pt.y - minPt.y()) / voxelResolution) + pad,
            static_cast<int>((pt.z - minPt.z()) / voxelResolution) + pad);
    };

    std::vector<Eigen::Vector3i> occupiedList;
    occupiedList.reserve(inputCloud->points.size());
    for (const auto& pt : inputCloud->points) {
        auto v = posToGrid(pt);
        grid[idx(v.x(), v.y(), v.z())] = VOX_OCCUPIED;
        occupiedList.push_back(v);
    }

    // STEP 1: DILATION
    int radiusVoxel =
        static_cast<int>(std::ceil(closingRadius / voxelResolution));
    for (const auto& v : occupiedList) {
        for (int dx = -radiusVoxel; dx <= radiusVoxel; ++dx) {
            for (int dy = -radiusVoxel; dy <= radiusVoxel; ++dy) {
                for (int dz = -radiusVoxel; dz <= radiusVoxel; ++dz) {
                    if (dx * dx + dy * dy + dz * dz <=
                        radiusVoxel * radiusVoxel) {
                        int nx = v.x() + dx, ny = v.y() + dy, nz = v.z() + dz;
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h && nz >= 0 &&
                            nz < d) {
                            if (grid[idx(nx, ny, nz)] == VOX_EMPTY) {
                                grid[idx(nx, ny, nz)] = VOX_DILATED;
                            }
                        }
                    }
                }
            }
        }
    }

    // STEP 2: FLOOD FILL
    std::queue<Eigen::Vector3i> q;
    q.push(Eigen::Vector3i(0, 0, 0));
    grid[idx(0, 0, 0)] = VOX_EXTERIOR;

    int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                      {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
    while (!q.empty()) {
        auto curr = q.front();
        q.pop();

        for (auto dir : dirs) {
            int nx = curr.x() + dir[0];
            int ny = curr.y() + dir[1];
            int nz = curr.z() + dir[2];

            if (nx >= 0 && nx < w && ny >= 0 && ny < h && nz >= 0 && nz < d) {
                if (grid[idx(nx, ny, nz)] == VOX_EMPTY) {
                    grid[idx(nx, ny, nz)] = VOX_EXTERIOR;
                    q.push(Eigen::Vector3i(nx, ny, nz));
                }
            }
        }
    }

    // STEP 3: STRICT SPHERICAL EXTRACTION
    int searchRadius = radiusVoxel + 1;
    for (size_t i = 0; i < inputCloud->points.size(); ++i) {
        auto v = occupiedList[i];
        bool touchesExterior = false;

        for (int dx = -searchRadius; dx <= searchRadius && !touchesExterior;
             ++dx) {
            for (int dy = -searchRadius; dy <= searchRadius && !touchesExterior;
                 ++dy) {
                for (int dz = -searchRadius;
                     dz <= searchRadius && !touchesExterior; ++dz) {
                    // Precision Sphere Math applied here
                    if (dx * dx + dy * dy + dz * dz <=
                        searchRadius * searchRadius) {
                        int nx = v.x() + dx, ny = v.y() + dy, nz = v.z() + dz;
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h && nz >= 0 &&
                            nz < d) {
                            if (grid[idx(nx, ny, nz)] == VOX_EXTERIOR) {
                                touchesExterior = true;
                            }
                        }
                    }
                }
            }
        }

        if (touchesExterior) {
            outputCloud->points.push_back(inputCloud->points[i]);
        }
    }

    outputCloud->width = outputCloud->points.size();
    outputCloud->height = 1;
    return outputCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::extractTeethRegion(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, bool isFullJaw,
    float extractionThickness) {
    if (!inputCloud || inputCloud->empty()) {
        qWarning() << "Input cloud empty.";
        return inputCloud;
    }

    // ==========================================================
    // PHASE 1: Modality-Tuned Processing
    // ==========================================================
    // CBCT gets aggressive 1.5mm sanding; STL gets high-fidelity 1.0mm
    // downsampling
    float leafParam = isFullJaw ? 1.5f : 1.0f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(leafParam, leafParam, leafParam);
    grid.setInputCloud(inputCloud);
    grid.filter(*downsampledCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothProxy(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setComputeNormals(false);
    mls.setInputCloud(downsampledCloud);
    mls.setPolynomialOrder(1);
    mls.setSearchMethod(mls_tree);
    mls.setSearchRadius(isFullJaw ? 4.0 : 1.5);
    mls.process(*smoothProxy);

    // ==========================================================
    // PHASE 2: The "Disposable Compass" Quarantine
    // ==========================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr coreCloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Vector4f global_centroid;
    pcl::compute3DCentroid(*smoothProxy, global_centroid);
    float coreRadiusSq = 45.0f * 45.0f;

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
        coreCloud = smoothProxy;  // Failsafe
    }

    // ==========================================================
    // PHASE 3: Curvature & RANSAC Plane Target Lock
    // ==========================================================
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(coreCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(isFullJaw ? 4.0 : 2.0);

    pcl::PointCloud<pcl::PointNormal>::Ptr normals(
        new pcl::PointCloud<pcl::PointNormal>());
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roughCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    float curveThreshold = isFullJaw ? 0.04f : 0.02f;

    for (size_t i = 0; i < coreCloud->points.size(); ++i) {
        if (normals->points[i].curvature > curveThreshold) {
            roughCloud->points.push_back(coreCloud->points[i]);
        }
    }
    if (roughCloud->points.size() < 100) roughCloud = coreCloud;

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
    // PHASE 4: Level the FULL Original Cloud
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr leveledCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Applying the perfect RANSAC compass matrix to the ORIGINAl RAW cloud
    // (molars are safe!)
    pcl::transformPointCloud(*inputCloud, *leveledCloud,
                             transform_to_canonical);

    // ==========================================================
    // PHASE 5: Center of Mass Gravity Polarity (Density-Immunized)
    // ==========================================================

    // High-resolution teeth surfaces create immense point density, artificially
    // dragging the centroid towards Z=0 and blinding the polarity check to the
    // massive palate roof. We apply spatial normalization (Voxel Grid) to
    // ensure sheer anatomical volume dictates gravity.
    pcl::PointCloud<pcl::PointXYZ>::Ptr balancedCloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> densityBalancer;
    densityBalancer.setLeafSize(5.0f, 5.0f, 5.0f);
    densityBalancer.setInputCloud(leveledCloud);
    densityBalancer.filter(*balancedCloud);

    Eigen::Vector4f rigid_centroid;
    pcl::compute3DCentroid(*balancedCloud, rigid_centroid);

    // Because the roots/palate vault dictate the true volumetric center of
    // mass, if rigid_centroid > 0, the jawbone is pointing UP (Maxillary
    // Anatomy).
    if (rigid_centroid.z() > 0) {
        Eigen::Matrix4f flip = Eigen::Matrix4f::Identity();
        flip(1, 1) =
            -1.0f;  // Flipper preserving Right-Handedness mathematically
        flip(2, 2) = -1.0f;
        transform_to_canonical = flip * transform_to_canonical;
        pcl::transformPointCloud(*inputCloud, *leveledCloud,
                                 transform_to_canonical);
    }

    // ==========================================================
    // PHASE 6: Pristine Anatomical Crown Extractor
    // ==========================================================
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(leveledCloud);
    pass.setFilterFieldName("z");
    // CBCTs need a deeper bite (12mm) to clear the jawbone edge. STLs only need
    // crowns (8mm).
    float safeCrownHeight = extractionThickness > 0.1f
                                ? extractionThickness
                                : (isFullJaw ? 12.0f : 8.0f);
    // FIXED: The Occlusal plane was already leveled to exactly Z = 0.
    // The roots/gums project downwards into negative Z.
    // We statically extract relative to Z=0 to prevent jaw hinges (condyles)
    // from hijacking the crop box. A +5.0f margin is allowed to include biting
    // cusps that protrude slightly above the planar fit.
    pass.setFilterLimits(-safeCrownHeight, 5.0f);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::extractByProximityMask(
    pcl::PointCloud<pcl::PointXYZ>::Ptr maskCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetSolidCloud, float searchRadius) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractedCloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    // Safety check
    if (!maskCloud || maskCloud->empty() || !targetSolidCloud ||
        targetSolidCloud->empty()) {
        qWarning() << "Invalid clouds provided to Proximity Extractor.";
        return extractedCloud;
    }

    // 1. Build KD-Tree Dict on the High-Resolution Solid Cloud
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(targetSolidCloud);

    // 2. Prevent duplicate points using an unordered_set (Massive Speed Gain)
    std::unordered_set<int> uniqueIndices;

    // 3. Cast the 3D 'Shrink-wrap' net from the Mask onto the Solid
    for (const auto& pt : maskCloud->points) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusDistance;

        // If the solid point is near the mask point, capture it
        if (kdtree.radiusSearch(pt, searchRadius, pointIdxRadiusSearch,
                                pointRadiusDistance) > 0) {
            for (int idx : pointIdxRadiusSearch) {
                uniqueIndices.insert(idx);
            }
        }
    }

    // 4. Convert gathered indices to strictly ordered PCL format
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices.assign(uniqueIndices.begin(), uniqueIndices.end());
    std::sort(inliers->indices.begin(), inliers->indices.end());

    // 5. Mathematically extract the flawless geometry
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(targetSolidCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);  // Retain intercepted points
    extract.filter(*extractedCloud);

    return extractedCloud;
}

vtkSmartPointer<vtkMatrix4x4> RegistrationModel::computeTransform(
    vtkSmartPointer<vtkPolyData> sourceStl,
    vtkSmartPointer<vtkPolyData> targetEnamel,
    vtkSmartPointer<vtkPolyData> targetEntireJaw) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSource = convertVtkToPcl(sourceStl);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclEnamel =
        convertVtkToPcl(targetEnamel);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclEntireJaw =
        convertVtkToPcl(targetEntireJaw);

    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/initial_enamel.ply",
        *pclEnamel);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/initial_entire_jaw.ply",
        *pclEntireJaw);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/initial_source.ply",
        *pclSource);

    vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
    transform->Identity();

    if (pclSource->points.empty() || pclEnamel->points.empty()) {
        vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
        transform->Identity();
        return transform;
    }

    qDebug() << "sealing internals";
    float closingRadius = 3.0f;
    float morphRes = 0.5f;

    auto morphTargetEnamel =
        applyMorphologicalClosing(pclEnamel, closingRadius, morphRes);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/morph_enamel.ply",
        *morphTargetEnamel);
    auto morphTargetEntireJaw =
        applyMorphologicalClosing(pclEntireJaw, 3.0f, 0.5f);

    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/morph_entire_jaw.ply",
        *morphTargetEntireJaw);

    vtkSmartPointer<vtkPolyData> croppedSourceMesh =
        cropStlInVtk(sourceStl, 10.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCroppedPcl =
        convertVtkToPcl(croppedSourceMesh);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/new_cropped_source.ply",
        *newCroppedPcl);

    // auto croppedSource = extractTeethRegion(pclSource, false, 8.0f);
    auto croppedTarget = extractTeethRegion(morphTargetEnamel, true, 8.0f);
    // pcl::io::savePLYFileASCII(
    //     "C:/Users/igrs/Desktop/Aswin/ImageReg_output/cropped_source.ply",
    //     *croppedSource);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/cropped_target.ply",
        *croppedTarget);

    // A. preprocessing stage
    float voxelLeafSize = 0.8f;
    float normalRadius = 2.0f;
    float featureRadius = 4.5f;

    auto sourceDown = downsampleCloud(newCroppedPcl, voxelLeafSize);
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
    // auto icpSource = extractTeethRegion(pclSource, false, 3.0f);
    vtkSmartPointer<vtkPolyData> restrictSourceMesh =
        cropStlInVtk(sourceStl, 3.0f);
    auto icpSource = convertVtkToPcl(restrictSourceMesh);
    auto icpTarget = extractTeethRegion(pclEnamel, true, 3.0f);

    qDebug() << "Generating high-fidelity point to plane normal map...";
    float highFidelityNormalRadius = 1.5f;
    auto fineSourceNormals =
        estimateNormals(newCroppedPcl, highFidelityNormalRadius);
    auto fineTargetNormals =
        estimateNormals(croppedTarget, highFidelityNormalRadius);

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

vtkSmartPointer<vtkPolyData> RegistrationModel::cropStlInVtk(
    vtkSmartPointer<vtkPolyData> inputMesh, float extractionThickness) {
    if (!inputMesh || inputMesh->GetNumberOfPoints() == 0) return inputMesh;

    // 1. Enforce True Triangles and Outward Biological Normals
    vtkSmartPointer<vtkTriangleFilter> triangles =
        vtkSmartPointer<vtkTriangleFilter>::New();
    triangles->SetInputData(inputMesh);
    triangles->Update();

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator =
        vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(triangles->GetOutput());
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->ComputePointNormalsOff();
    normalGenerator->ConsistencyOn();  // Enforces the physical "Inside vs
                                       // Outside" boundary
    normalGenerator->SplittingOff();
    normalGenerator->Update();

    vtkPolyData* meshWithNormals = normalGenerator->GetOutput();
    vtkFloatArray* cellNormals = vtkFloatArray::SafeDownCast(
        meshWithNormals->GetCellData()->GetNormals());

    // 2. The Gauss Dome Integration (Area-Weighted Normal Summing)
    // Finding the absolute "UP" direction by taking advantage of the hollow
    // shell physics
    double globalVector[3] = {0.0, 0.0, 0.0};

    for (vtkIdType i = 0; i < meshWithNormals->GetNumberOfCells(); ++i) {
        vtkCell* cell = meshWithNormals->GetCell(i);
        if (cell->GetCellType() == VTK_TRIANGLE) {
            vtkPoints* pts = cell->GetPoints();
            double p0[3], p1[3], p2[3];
            pts->GetPoint(0, p0);
            pts->GetPoint(1, p1);
            pts->GetPoint(2, p2);

            // Area-weighting ensures massive flat bases do not overpower the
            // highly faceted teeth
            double area = vtkTriangle::TriangleArea(p0, p1, p2);
            double normal[3];
            cellNormals->GetTuple(i, normal);

            globalVector[0] += normal[0] * area;
            globalVector[1] += normal[1] * area;
            globalVector[2] += normal[2] * area;
        }
    }

    double magnitude = vtkMath::Norm(globalVector);

    // Safety Fallback: If STL is a perfect solid block, sum is 0 (Gauss's
    // Theorem).
    if (magnitude < 1e-5) {
        qWarning() << "Mesh is perfectly closed. Gauss Dome integration "
                      "failed. Using strict Z baseline.";
        globalVector[0] = 0.0;
        globalVector[1] = 0.0;
        globalVector[2] = 1.0;
        magnitude = 1.0;
    }

    globalVector[0] /= magnitude;
    globalVector[1] /= magnitude;
    globalVector[2] /= magnitude;

    // 3. Mathematical Alignment Matrix (Leveling the True Orientation)
    Eigen::Vector3f globalV(globalVector[0], globalVector[1], globalVector[2]);
    Eigen::Vector3f worldZ(0.0f, 0.0f, 1.0f);

    float angle = std::acos(globalV.dot(worldZ));
    Eigen::Vector3f rotationAxis = globalV.cross(worldZ);

    // Prevent Eigen NaN crashes if the scan is perfectly inverted (180deg) or
    // perfectly aligned (0deg)
    if (rotationAxis.norm() < 1e-5f) {
        rotationAxis =
            Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // Default to X-axis flip
    } else {
        rotationAxis.normalize();
    }

    vtkSmartPointer<vtkTransform> transform =
        vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();
    if (std::abs(globalV.dot(worldZ)) < 0.999f) {
        transform->RotateWXYZ(angle * 180.0 / vtkMath::Pi(), rotationAxis[0],
                              rotationAxis[1], rotationAxis[2]);
    }

    vtkSmartPointer<vtkTransformPolyDataFilter> alignFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    alignFilter->SetInputData(meshWithNormals);
    alignFilter->SetTransform(transform);
    alignFilter->Update();

    vtkSmartPointer<vtkPolyData> alignedMesh = alignFilter->GetOutput();

    // ==========================================================
    // 3.5 Topological Border Polarity Fix (Density-Independent)
    // ==========================================================
    // Rather than relying on point density, we rely on pure topology.
    // The only massive "open edge" in an intraoral optical scan is the
    // gum-line scanner boundary. The teeth are a watertight surface.
    vtkSmartPointer<vtkFeatureEdges> edgeExtractor =
        vtkSmartPointer<vtkFeatureEdges>::New();
    edgeExtractor->SetInputData(alignedMesh);
    edgeExtractor->BoundaryEdgesOn();  // Extract the open rim
    edgeExtractor->FeatureEdgesOff();
    edgeExtractor->NonManifoldEdgesOff();
    edgeExtractor->ManifoldEdgesOff();
    edgeExtractor->Update();

    vtkPolyData* boundaries = edgeExtractor->GetOutput();
    vtkIdType numBoundaryPts = boundaries->GetNumberOfPoints();

    // If numBoundaryPts == 0, it's a fully closed solid model, meaning
    // the Gauss Integration worked natively and we do not intervene.
    if (numBoundaryPts > 50) {
        double currentBounds[6];
        alignedMesh->GetBounds(currentBounds);
        double zMid = (currentBounds[4] + currentBounds[5]) / 2.0;

        double meanBoundaryZ = 0.0;
        for (vtkIdType i = 0; i < numBoundaryPts; ++i) {
            double p[3];
            boundaries->GetPoint(i, p);
            meanBoundaryZ += p[2];
        }
        meanBoundaryZ /= static_cast<double>(numBoundaryPts);

        // If the open boundary (the gum skirt) is physically higher than the
        // center of the bounding box, the teeth are definitively upside down.
        if (meanBoundaryZ > zMid) {
            vtkSmartPointer<vtkTransform> flipTransform =
                vtkSmartPointer<vtkTransform>::New();
            flipTransform->PostMultiply();
            flipTransform->RotateX(180.0);  // 180° Right-Handed inversion

            vtkSmartPointer<vtkTransformPolyDataFilter> flipFilter =
                vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            flipFilter->SetInputData(alignedMesh);
            flipFilter->SetTransform(flipTransform);
            flipFilter->Update();

            alignedMesh = flipFilter->GetOutput();

            // Mathematically append for Phase 6 unwrap
            transform->RotateX(180.0);
        }
    }

    // ==========================================================
    // 4. "Dropping the Plane" to Isolate Teeth for PCA
    // ==========================================================
    double bounds[6];  // [xmin, xmax, ymin, ymax, zmin, zmax]
    alignedMesh->GetBounds(bounds);

    double const initialMaxZ = bounds[5];
    double const zSpan = bounds[5] - bounds[4];

    vtkSmartPointer<vtkPlane> probePlane = vtkSmartPointer<vtkPlane>::New();

    // DYNAMIC EXPANSION: Drop 20mm from top (but max 60% of total height)
    // This feeds the FULL 60x50mm U-shaped dental arch to PCA, preventing it
    // from mathematically collapsing if only the front incisors were caught.
    double safeClipZ = std::max(initialMaxZ - 20.0, bounds[5] - (zSpan * 0.6));

    probePlane->SetOrigin(0.0, 0.0, safeClipZ);
    probePlane->SetNormal(0.0, 0.0, 1.0);

    vtkSmartPointer<vtkClipPolyData> topsClipper =
        vtkSmartPointer<vtkClipPolyData>::New();
    topsClipper->SetInputData(alignedMesh);
    topsClipper->SetClipFunction(probePlane);
    topsClipper->Update();

    vtkPolyData* topsMesh = topsClipper->GetOutput();
    vtkIdType const numTops = topsMesh->GetNumberOfPoints();

    // Isolate the True Occlusal Tilt via minimal variance vector (PCA)
    if (numTops > 50) {
        Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
        for (vtkIdType i = 0; i < numTops; ++i) {
            double p[3];
            topsMesh->GetPoint(i, p);
            centroid += Eigen::Vector3f(static_cast<float>(p[0]),
                                        static_cast<float>(p[1]),
                                        static_cast<float>(p[2]));
        }
        centroid /= static_cast<float>(numTops);

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (vtkIdType i = 0; i < numTops; ++i) {
            double p[3];
            topsMesh->GetPoint(i, p);
            Eigen::Vector3f pt(static_cast<float>(p[0]),
                               static_cast<float>(p[1]),
                               static_cast<float>(p[2]));
            Eigen::Vector3f const centered = pt - centroid;
            covariance += centered * centered.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca(covariance);

        // The minimal variance axis locates the orthogonal normal to the
        // U-shaped arch
        Eigen::Vector3f trueOcclusalNormal = pca.eigenvectors().col(0);

        if (trueOcclusalNormal.z() < 0) {
            trueOcclusalNormal =
                -trueOcclusalNormal;  // Enforce Biological UP (+Z)
        }

        Eigen::Vector3f const targetWorldZ(0.0f, 0.0f, 1.0f);
        float const correctiveAngle =
            std::acos(trueOcclusalNormal.dot(targetWorldZ));
        Eigen::Vector3f tiltAxis = trueOcclusalNormal.cross(targetWorldZ);

        // Sub-millimeter tilt alignment execution
        if (tiltAxis.norm() > 1e-5f) {
            tiltAxis.normalize();

            transform->Translate(-centroid.x(), -centroid.y(), -centroid.z());
            transform->RotateWXYZ(correctiveAngle * 180.0 / vtkMath::Pi(),
                                  tiltAxis.x(), tiltAxis.y(), tiltAxis.z());
            transform->Translate(centroid.x(), centroid.y(), centroid.z());

            alignFilter->Update();
            alignedMesh = alignFilter->GetOutput();
        }
    }

    // ==========================================================
    // 5. Extreme Precision Peak Slicing (The Floating Sheet)
    // ==========================================================
    alignedMesh->GetBounds(bounds);
    double const finalMaxZ = bounds[5];
    vtkSmartPointer<vtkPlane> clipPlane = vtkSmartPointer<vtkPlane>::New();
    // Suspend the mathematical sheet downward from the fully-leveled,
    // synchronized cusp height
    clipPlane->SetOrigin(0.0, 0.0, finalMaxZ - extractionThickness);
    clipPlane->SetNormal(0.0, 0.0, 1.0);
    vtkSmartPointer<vtkClipPolyData> clipper =
        vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputData(alignedMesh);
    clipper->SetClipFunction(clipPlane);
    clipper->Update();
    // ==========================================================
    // 6. Restore Absolute World Coordinates (Seamless Integration)
    // ==========================================================
    vtkSmartPointer<vtkTransform> invTransform =
        vtkSmartPointer<vtkTransform>::New();
    // The mathematical matrix natively unwraps BOTH the baseline Gauss
    // alignment AND the PCA tilt concurrently
    invTransform->SetMatrix(transform->GetMatrix());
    invTransform->Inverse();
    vtkSmartPointer<vtkTransformPolyDataFilter> restoreFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    restoreFilter->SetInputData(clipper->GetOutput());
    restoreFilter->SetTransform(invTransform);
    restoreFilter->Update();
    return restoreFilter->GetOutput();
}
void RegistrationModel::saveDiagnosticCrop(
    vtkSmartPointer<vtkPolyData> inputStl, const QString& outputPath) {
    if (!inputStl || inputStl->GetNumberOfPoints() == 0) return;

    // Confined entirely inside the Math model
    vtkSmartPointer<vtkPolyData> croppedMesh = cropStlInVtk(inputStl, 8.0f);

    vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
    writer->SetFileName(outputPath.toUtf8().constData());
    writer->SetInputData(croppedMesh);
    writer->Write();
    qDebug() << "Diagnostic VTK Crop cleanly saved to:" << outputPath;
}
