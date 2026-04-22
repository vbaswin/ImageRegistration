#pragma once

#include <memory>

#include "RegistrationTypes.h"

class RegistrationModel;

namespace ImageRegistration {

class DentalRegistrationEngine {
   public:
    DentalRegistrationEngine();
    ~DentalRegistrationEngine();

    RegistrationResult registerStlToImage(const RegistrationInput& input,
                                          const RegistrationSettings& settings);

   private:
    std::unique_ptr<RegistrationModel> m_registrationModel;
};

}  // namespace ImageRegistration
