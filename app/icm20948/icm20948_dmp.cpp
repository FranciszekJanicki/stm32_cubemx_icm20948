#include "icm20948_dmp.hpp"

namespace ICM20948 {

    ICM20948_DMP::ICM20948_DMP(ICM20948&& base, DMPConfig const& dmp_config) noexcept :
        base_{std::forward<ICM20948>(base)}
    {
        this->initialize(dmp_config);
    }

    ICM20948_DMP::~ICM20948_DMP() noexcept
    {
        this->deinitialize();
    }

    void ICM20948_DMP::initialize(DMPConfig const& dmp_config) noexcept
    {}

    void ICM20948_DMP::deinitialize() noexcept
    {}

}; // namespace ICM20948