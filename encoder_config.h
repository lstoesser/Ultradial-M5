#pragma once

// Encoder configuration page. The selected encoder must be supported directly by
// ODrive Micro because it is used for commutation and dial position feedback.
namespace ultradial::encoder {
enum class ProfileId {
  ODriveMicroOnboardMa702, Amt23Spi, Fit1036IntegratedAs5600, Amt212bRs485,
};
enum class Bus { Onboard, Spi, I2c, Rs485 };

struct EncoderConfig {
  const char* name;
  Bus bus;
  bool supportedByOdriveMicro;
  bool isAbsolute;
  int resolutionBits;
  const char* odriveMode;
  const char* note;
};

constexpr EncoderConfig kOdriveMicroOnboardMa702 = {
    "ODrive Micro onboard MA702", Bus::Onboard, true, true, 14,
    "EncoderId.ONBOARD_ENCODER0",
    "Mount a diametric magnet on the FIT1036 shaft, aligned to the Micro.",
};
constexpr EncoderConfig kAmt23Spi = {
    "CUI AMT23 SPI", Bus::Spi, true, true, 14,
    "EncoderId.SPI_ENCODER0 / SpiEncoderMode.CUI_AMT23",
    "Supported external SPI alternative. This is not the AMT212B.",
};
constexpr EncoderConfig kFit1036IntegratedAs5600 = {
    "FIT1036 integrated AS5600", Bus::I2c, false, true, 12,
    "not supported by ODrive Micro",
    "Useful to an I2C host, but it cannot be the Micro's commutation encoder.",
};
constexpr EncoderConfig kAmt212bRs485 = {
    "CUI AMT212B", Bus::Rs485, false, true, 12,
    "not supported by ODrive Micro",
    "AMT212B is RS-485, not SPI; use it with an ODrive Pro/S1, not a Micro.",
};

// ---------- Change this one line to select an encoder ----------
constexpr ProfileId kSelectedProfile = ProfileId::ODriveMicroOnboardMa702;

// Keep this as an expression: PlatformIO's Arduino toolchain defaults to C++11,
// where constexpr functions cannot contain a switch statement.
constexpr EncoderConfig kActive =
    kSelectedProfile == ProfileId::ODriveMicroOnboardMa702 ? kOdriveMicroOnboardMa702 :
    kSelectedProfile == ProfileId::Amt23Spi ? kAmt23Spi :
    kSelectedProfile == ProfileId::Fit1036IntegratedAs5600 ?
        kFit1036IntegratedAs5600 :
    kAmt212bRs485;
static_assert(kActive.supportedByOdriveMicro,
              "Selected encoder is not supported by ODrive Micro. See encoder_config.h.");
}  // namespace ultradial::encoder
