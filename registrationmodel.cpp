// #ifdef _MSC_VER
// #include <immintrin.h>
// #include <intrin.h>
// #endif

#include "registrationmodel.h"
#include <algorithm>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
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
pcl::PointCloud<pcl::PointXYZ>::Ptr RegistrationModel::extractTeethRegion(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    if (!inputCloud || inputCloud->empty()) {
        qWarning() << "Input cloud empty";
        return inputCloud;
    }

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(inputCloud);

    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector4f centroid = pca.getMean();

    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform.block<3, 3>(0, 0) = eigen_vectors.transpose();

    transform.block<3, 1>(0, 3) = -1.0f * (eigen_vectors.transpose() * centroid.head<3>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcaAlignedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*inputCloud, *pcaAlignedCloud, transform);

    // 1D z axis point density histogram
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*pcaAlignedCloud, min_pt, max_pt);

    float z_range = max_pt.z - min_pt.z;

    if (z_range <= 0.0f || std::isnan(z_range)) {
        qWarning() << "CRITICAL ERROR: Z-range is invalid:" << z_range
                   << ". Returning original cloud.";
        return inputCloud;
    }

    int num_bins = 60; // granularity of the slice
    float bin_size = z_range / static_cast<float>(num_bins);
    std::vector<int> histogram(num_bins, 0);

    // populate histogram
    for (const auto &point : pcaAlignedCloud->points) {
        if (std::isnan(point.z))
            continue;
        int bin_idx = std::max(0,
                               std::min(num_bins - 1,
                                        static_cast<int>((point.z - min_pt.z) / bin_size)));
        histogram[bin_idx]++;
    }

    auto peak_iter = std::max_element(histogram.begin(), histogram.end());
    int peak_idx = static_cast<int>(std::distance(histogram.begin(), peak_iter));

    float peak_z_center = min_pt.z + (peak_idx * bin_size) + (bin_size / 2.0f);
    // float extraction_thickness = 18.0f;

    float dynamic_thickness = z_range * 0.25f;
    float min_limit = peak_z_center - (dynamic_thickness / 2.0f);
    float max_limit = peak_z_center + (dynamic_thickness / 2.0f);

    // Output Telemetry to trace coordinate scales
    qDebug() << "=== Slicing Diagnostics ===";
    qDebug() << "Total Z Range:" << z_range << "from" << min_pt.z << "to" << max_pt.z;
    qDebug() << "Peak Bin Index:" << peak_idx << "with density" << *peak_iter << "points";
    qDebug() << "Dynamic PassThrough Limits:" << min_limit << "to" << max_limit;

    pcl::PointCloud<pcl::PointXYZ>::Ptr slicedPcaCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcaAlignedCloud);
    pass.setFilterFieldName("z");
    // pass.setFilterLimits(peak_z_center = (extraction_thickness / 2.0f),
    //                      peak_z_center + (extraction_thickness / 2.0f));
    pass.setFilterLimits(min_limit, max_limit);
    pass.filter(*slicedPcaCloud);

    if (slicedPcaCloud->points.empty()) {
        qWarning() << "Filter obliterated all points. Sclae error? Returning original cloud.";
        return inputCloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalWorldCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*slicedPcaCloud, *finalWorldCloud, transform.inverse());

    qDebug() << "Extracted teeth region. Point count reduced from: " << inputCloud->points.size()
             << " to " << finalWorldCloud->points.size();

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
    float voxelLeafSize = 2.0f;
    float normalRadius = 4.0f;

    auto sourceDown = downsampleCloud(pclSource, voxelLeafSize);
    auto targetDown = downsampleCloud(pclTarget, voxelLeafSize);

    auto sourceNormals = estimateNormals(sourceDown, normalRadius);
    auto targetNormals = estimateNormals(targetDown, normalRadius);
    // auto sourceNormals = estimateNormals(pclSource, normalRadius);
    // auto targetNormals = estimateNormals(pclTarget, normalRadius);

    // B. orientation invariant filtering - curvature extraction

    // helper lambda to extract curvature points
    /*
    auto extractHighCurvature = [](pcl::PointCloud<pcl::PointNormal>::Ptr inputNormalCloud,
                                   float curvatureThreshold) {
        pcl::PointIndices::Ptr highCurvatureIndices(new pcl::PointIndices());
        // Curvature value is directly computed by NormalEstimationOMP
        // 0.0 = completely flat, > 0.05 = sharp edges/bumps
        for (size_t i = 0; i < inputNormalCloud->points.size(); ++i) {
            if (inputNormalCloud->points[i].curvature > curvatureThreshold) {
                highCurvatureIndices->indices.push_back(i);
            }
        }

        pcl::PointCloud<pcl::PointNormal>::Ptr featuredCloud(
            new pcl::PointCloud<pcl::PointNormal>());

        pcl::ExtractIndices<pcl::PointNormal> extract;
        extract.setInputCloud(inputNormalCloud);
        extract.setIndices(highCurvatureIndices);
        extract.setNegative(false); // keep only high curvature points
        extract.filter(*featuredCloud);

        if (featuredCloud->points.size() < 100) {
            qWarning() << "Curvature threshold too high. Returning original cloud";
            return inputNormalCloud;
        }
        return featuredCloud;
    };
    */

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

    // D. ransac global alignment
    qDebug() << "Executing RANSAC  ...";
    float maxCorrespDist = 15.0f;
    vtkSmartPointer<vtkMatrix4x4> ransacTransform = performRANSAC(sourceNormals,
                                                                  targetNormals,
                                                                  sourceFPFH,
                                                                  targetFPFH,
                                                                  maxCorrespDist);

    qDebug() << "RANSAC  Execution complete!";

    return ransacTransform;
}
