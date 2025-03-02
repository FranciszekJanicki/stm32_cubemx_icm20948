#ifndef ICM20948_DMP_HPP
#define ICM20948_DMP_HPP

#include "icm20948.hpp"
#include "icm20948_dmp_config.hpp"
#include "icm20948_dmp_img.hpp"
#include "icm20948_dmp_registers.hpp"

namespace ICM20948 {

    struct ICM20948_DMP {
    public:
        ICM20948_DMP() noexcept = default;
        ICM20948_DMP(ICM20948&& base, DMPConfig const& dmp_config) noexcept;

        ICM20948_DMP(ICM20948_DMP const& other) = delete;
        ICM20948_DMP(ICM20948_DMP&& other) noexcept = default;

        ICM20948_DMP& operator=(ICM20948_DMP const& other) = delete;
        ICM20948_DMP& operator=(ICM20948_DMP&& other) noexcept = default;

        ~ICM20948_DMP() noexcept;

    private:
        void initialize(DMPConfig const& dmp_config) noexcept;
        void deinitialize() noexcept;

        ICM20948 base_{};
    };

}; // namespace ICM20948

#endif // ICM20948_DMP_HPP