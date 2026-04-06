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
#include <cstdint>
#include <pcl/filters/impl/extract_indices.hpp>
#include <queue>
#include <vector>

#include "itk_volumemath.h"

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
    // We apply this to BOTH STL and CBCT. It safely amputates ramus hinges and
    // ragged STL palate borders, giving RANSAC a pristine U-shaped front arch
    // to aim at.
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

    qDebug() << "sealing internals";
    float closingRadius = 3.0f;
    float morphRes = 0.5f;

    auto solidTarget =
        applyMorphologicalClosing(pclTarget, closingRadius, morphRes);
    pcl::io::savePLYFileASCII(
        "C:/Users/igrs/Desktop/Aswin/ImageReg_output/solid_target.ply",
        *solidTarget);

    auto croppedSource = extractTeethRegion(pclSource, false, 8.0f);
    auto croppedTarget = extractTeethRegion(solidTarget, true, 8.0f);
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
vtkSmartPointer<vtkMatrix4x4> RegistrationModel::computeVolumetricTransform(
    vtkSmartPointer<vtkPolyData> sourceStl,
    vtkSmartPointer<vtkImageData> targetCbct) {
    // Pass execution across the compiler barrier to the insulated ITK
    // Translation Unit
    return ITKVolumeMath::executeDistanceFieldRegistration(sourceStl,
                                                           targetCbct);
}
