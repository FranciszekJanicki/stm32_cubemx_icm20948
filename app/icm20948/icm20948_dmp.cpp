#include "icm20948_dmp.hpp"
#include "icm20948.hpp"
#include "icm20948_dmp_config.hpp"
#include "icm20948_dmp_registers.hpp"

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
    {
        // this->base_.device_wake_up();

        // this->base_.set_sleep_enabled(false);
        // this->set_memory_bank(0x10, true, true);
        // this->set_memory_start_address(0x06);
        // this->set_memory_bank(0, false, false);
        // this->get_otp_bank_valid();

        // this->base_.set_slave_address(0, 0x7F);
        // this->base_.set_i2c_master_mode_enabled(false);
        // this->base_.set_slave_address(0, 0x68);
        // this->base_.reset_i2c_master();
        // this->base_.set_clock_source(Clock::PLL_ZGYRO);
        // this->set_int_dmp_enabled(true);
        // this->base_.set_int_fifo_overflow_enabled(true);
        // this->base_.set_sampling_rate(200, DLPF::BW_42);
        // this->base_.set_external_frame_sync(ExtSync::TEMP_OUT_L);
        // this->base_.set_dlpf_mode(DLPF::BW_42);
        // this->base_.set_full_scale_gyro_range(GyroRange::GYRO_FS_2000);
        // this->write_memory_block(dmp_memory.data(), dmp_memory.size(), 0x00, 0x00);

        // std::array<std::uint8_t, 2UL> dmp_update{0x00, 0x01};
        // this->write_memory_block(dmp_update.data(), 0x02, 0x02, 0x16);
        // this->set_dmp_config1(0x03);
        // this->set_dmp_config2(0x00);
        // this->set_otp_bank_valid(false);

        // this->base_.set_motion_detection_threshold(2);
        // this->base_.set_zero_motion_detection_threshold(156);
        // this->base_.set_motion_detection_duration(80);
        // this->base_.set_zero_motion_detection_duration(0);
        // this->base_.set_fifo_enabled(true);
        // this->reset_dmp();
        // this->set_dmp_enabled(false);
        // this->base_.get_int_status();
        // this->base_.reset_fifo();
        // this->set_dmp_enabled(true);
    }

    void ICM20948_DMP::deinitialize() noexcept
    {}

    std::array<std::uint8_t, 42UL> ICM20948_DMP::get_dmp_packet() const noexcept
    {
        // std::array<std::uint8_t, 42UL> dmp_packet{};

        // if (this->get_int_dmp_status()) {
        //     auto fifo_count = this->base_.get_fifo_count();
        //     if (fifo_count == FIFO_MAX_COUNT) {
        //         this->base_.reset_fifo();
        //     }

        //     while (fifo_count < dmp_packet.size()) {
        //     }

        //     for (auto i{0}; i < fifo_count / dmp_packet.size(); ++i) {
        //         this->base_.get_fifo_bytes(dmp_packet.data(), dmp_packet.size());
        //     }
        // }

        // return dmp_packet;
    }

    Quat3D<std::int16_t> ICM20948_DMP::get_quaternion_raw() const noexcept
    {
        // auto packet = this->get_dmp_packet();
        // return Quat3D<std::int16_t>{(static_cast<std::int16_t>(packet[0]) << 8) |
        // static_cast<std::int16_t>(packet[1]),
        //                             (static_cast<std::int16_t>(packet[4]) << 8) |
        //                             static_cast<std::int16_t>(packet[5]), (static_cast<std::int16_t>(packet[8]) << 8)
        //                             | static_cast<std::int16_t>(packet[9]), (static_cast<std::int16_t>(packet[12]) <<
        //                             8) |
        //                                 static_cast<std::int16_t>(packet[13])};
    }

    Quat3D<float> ICM20948_DMP::get_quaternion_scaled() const noexcept
    {
        return static_cast<Quat3D<float>>(this->get_quaternion_raw()) / this->base_.accel_scale_;
    }

    Vec3D<float> ICM20948_DMP::get_gravity() const noexcept
    {
        return quaternion_to_gravity(this->get_quaternion_scaled());
    }

    float ICM20948_DMP::get_roll() const noexcept
    {
        return quaternion_to_roll(this->get_quaternion_scaled());
    }

    float ICM20948_DMP::get_pitch() const noexcept
    {
        return quaternion_to_pitch(this->get_quaternion_scaled());
    }

    float ICM20948_DMP::get_yaw() const noexcept
    {
        return quaternion_to_yaw(this->get_quaternion_scaled());
    }

    Vec3D<float> ICM20948_DMP::get_roll_pitch_yaw() const noexcept
    {
        return quaternion_to_roll_pitch_yaw(this->get_quaternion_scaled());
    }

    void ICM20948_DMP::set_memory_bank(std::uint8_t const bank,
                                       bool const prefetch_enabled,
                                       bool const user_bank) const noexcept
    {
        // std::uint8_t data = bank & 0x1F;
        // if (user_bank)
        //     data |= 0x20;
        // if (prefetch_enabled)
        //     data |= 0x40;
        // this->base_.write_byte(std::to_underlying(RA::MEM_BANK_SEL), data);
    }

    void ICM20948_DMP::set_memory_start_address(std::uint8_t const address) const noexcept
    {
        // this->base_.write_byte(std::to_underlying(RA::MEM_START_ADDR), address);
    }

    std::uint8_t ICM20948_DMP::read_memory_byte() const noexcept
    {
        // return this->base_.read_byte(std::to_underlying(RA::MEM_R_W));
    }

    void ICM20948_DMP::write_memory_byte(std::uint8_t const data) const noexcept
    {
        // this->base_.write_byte(std::to_underlying(RA::MEM_R_W), data);
    }

    void ICM20948_DMP::read_memory_block(std::uint8_t* read_data,
                                         std::size_t const read_size,
                                         std::uint8_t bank,
                                         std::uint8_t address) const noexcept
    {
        // this->set_memory_bank(bank);
        // this->set_memory_start_address(address);

        // for (std::int16_t i = 0; i < read_size;) {
        //     std::uint8_t chunk_size = DMP_MEMORY_CHUNK_SIZE;

        //     if (i + chunk_size > read_size) {
        //         chunk_size = read_size - i;
        //     }
        //     if (chunk_size > 256 - address) {
        //         chunk_size = 256 - address;
        //     }

        //     this->base_.read_bytes(std::to_underlying(RA::MEM_R_W), read_data + i, chunk_size);
        //     i += chunk_size;
        //     address += chunk_size;

        //     if (i < read_size) {
        //         if (address == 0) {
        //             bank++;
        //         }
        //         this->set_memory_bank(bank);
        //         this->set_memory_start_address(address);
        //     }
        // }
    }

    void ICM20948_DMP::write_memory_block(std::uint8_t* write_data,
                                          std::size_t const write_size,
                                          std::uint8_t bank,
                                          std::uint8_t address) const noexcept
    {
        // this->set_memory_bank(bank);
        // this->set_memory_start_address(address);

        // for (std::int16_t i = 0; i < write_size;) {
        //     std::uint8_t chunk_size = DMP_MEMORY_CHUNK_SIZE;

        //     if (i + chunk_size > write_size) {
        //         chunk_size = write_size - i;
        //     }
        //     if (chunk_size > 256 - address) {
        //         chunk_size = 256 - address;
        //     }

        //     this->base_.write_bytes(std::to_underlying(RA::MEM_R_W), write_data + i, chunk_size);
        //     i += chunk_size;
        //     address += chunk_size;

        //     if (i < write_size) {
        //         if (address == 0) {
        //             bank++;
        //         }
        //         this->set_memory_bank(bank);
        //         this->set_memory_start_address(address);
        //     }
        // }
    }

    void ICM20948_DMP::write_dmp_configuration_set(std::uint8_t* write_data,
                                                   std::size_t const write_size) const noexcept
    {
        // for (std::int16_t i = 0; i < write_size;) {
        //     std::uint8_t bank = write_data[i++];
        //     std::uint8_t offset = write_data[i++];
        //     std::uint8_t length = write_data[i++];

        //     if (length > 0) {
        //         this->write_memory_block(write_data + i, length, bank, offset);
        //         i += length;
        //     } else {
        //         if (write_data[i++] == 0x01) {
        //             this->base_.write_byte(std::to_underlying(RA::INT_ENABLE), 0x32);
        //         }
        //     }
        // }
    }

}; // namespace ICM20948