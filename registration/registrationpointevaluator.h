#pragma once

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

#include <array>
#include <vector>

namespace ImageRegistration {

class RegistrationPointEvaluator {
   public:
    void savePoint(const std::array<double, 3>& point, bool isStl);
    void clearPoints();
    void setTransform(vtkMatrix4x4* transformMatrix);
    void calculateRMS() const;

   private:
    std::vector<std::array<double, 3>> m_stlPoints;
    std::vector<std::array<double, 3>> m_cbctPoints;
    vtkSmartPointer<vtkMatrix4x4> m_transformMatrix;
};

}  // namespace ImageRegistration
