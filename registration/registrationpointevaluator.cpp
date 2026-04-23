#include "RegistrationPointEvaluator.h"

#include <vtkNew.h>
#include <vtkTransform.h>

#include <QDebug>
#include <cmath>

namespace ImageRegistration {
namespace {

using Point3D = std::array<double, 3>;

Point3D transformPoint(const Point3D& point, vtkTransform* transform) {
    Point3D transformedPoint{};
    transform->TransformPoint(point.data(), transformedPoint.data());
    return transformedPoint;
}

double squaredDistance(const Point3D& a, const Point3D& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];

    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

void RegistrationPointEvaluator::savePoint(const std::array<double, 3>& point,
                                           bool isStl) {
    if (isStl) {
        m_stlPoints.push_back(point);
    } else {
        m_cbctPoints.push_back(point);
    }
}

void RegistrationPointEvaluator::clearPoints() {
    m_stlPoints.clear();
    m_cbctPoints.clear();
}

void RegistrationPointEvaluator::setTransform(vtkMatrix4x4* transformMatrix) {
    if (!transformMatrix) {
        m_transformMatrix = nullptr;
        return;
    }

    if (!m_transformMatrix) {
        m_transformMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    }

    m_transformMatrix->DeepCopy(transformMatrix);
}

void RegistrationPointEvaluator::calculateRMS() const {
    if (!m_transformMatrix) {
        qDebug() << "Skipping RMSE: STL to CBCT transform matrix is null";
        return;
    }

    if (m_stlPoints.size() != m_cbctPoints.size()) {
        qDebug() << "Skipping RMSE: STL and CBCT point counts do not match";
        return;
    }

    if (m_stlPoints.empty()) {
        qDebug() << "Skipping RMSE: no picked points available";
        return;
    }

    vtkNew<vtkTransform> stlToCbctTransform;
    stlToCbctTransform->SetMatrix(m_transformMatrix);

    double sumSquaredDistance = 0.0;

    for (size_t i = 0; i < m_stlPoints.size(); ++i) {
        const Point3D transformedStlPoint =
            transformPoint(m_stlPoints[i], stlToCbctTransform);

        const double pairSquaredDistance =
            squaredDistance(transformedStlPoint, m_cbctPoints[i]);

        qDebug() << "Pair:" << i + 1
                 << "error:" << std::sqrt(pairSquaredDistance);

        sumSquaredDistance += pairSquaredDistance;
    }

    const double rms = std::sqrt(sumSquaredDistance / m_stlPoints.size());
    qDebug() << "RMSE:" << rms;
}

}  // namespace ImageRegistration
