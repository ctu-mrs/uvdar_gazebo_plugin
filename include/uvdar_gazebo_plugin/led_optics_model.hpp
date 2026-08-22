#pragma once

#include <algorithm>
#include <cmath>

namespace uvdar_gazebo_plugin
{
namespace optics
{

struct CameraResponse
{
  // Physical optical chain. The aperture is represented by an equivalent
  // circular entrance pupil and transmission includes lens/filter losses.
  double exposure_us = 1000.0;
  double aperture_diameter_m = 0.001;
  double optical_transmission = 0.2;
  double quantum_efficiency = 0.3;
  double wavelength_nm = 395.0;
  double electrons_per_adu = 80.0;

  // Optical blur in sensor pixels.  Pixel integration and sensor clipping
  // naturally turn a strong unresolved source into a saturated central blob.
  double psf_sigma_px = 0.35;
};

inline double lambertianRelativeIntensity(double cos_angle, double order)
{
  if (!std::isfinite(cos_angle) || !std::isfinite(order)
      || cos_angle <= 0.0 || order < 0.0) {
    return 0.0;
  }

  // Relative to the on-axis intensity of a first-order Lambertian source.
  // The order-dependent normalization conserves total emitted power.
  return 0.5 * (order + 1.0) * std::pow(std::min(1.0, cos_angle), order);
}

inline double integratedSignalAdu(
    const CameraResponse &response,
    double power_w,
    double cos_angle,
    double lambertian_order,
    double distance_m,
    double gain = 1.0)
{
  if (!std::isfinite(power_w) || power_w <= 0.0
      || !std::isfinite(distance_m) || distance_m <= 0.0
      || !std::isfinite(gain) || gain <= 0.0
      || !std::isfinite(response.exposure_us)
      || response.exposure_us <= 0.0
      || !std::isfinite(response.aperture_diameter_m)
      || response.aperture_diameter_m <= 0.0
      || !std::isfinite(response.optical_transmission)
      || response.optical_transmission <= 0.0
      || response.optical_transmission > 1.0
      || !std::isfinite(response.quantum_efficiency)
      || response.quantum_efficiency <= 0.0
      || response.quantum_efficiency > 1.0
      || !std::isfinite(response.wavelength_nm)
      || response.wavelength_nm <= 0.0
      || !std::isfinite(response.electrons_per_adu)
      || response.electrons_per_adu <= 0.0) {
    return 0.0;
  }

  const double angular = lambertianRelativeIntensity(cos_angle, lambertian_order);
  if (angular <= 0.0) {
    return 0.0;
  }

  constexpr double kPlanckConstantJs = 6.62607015e-34;
  constexpr double kSpeedOfLightMPerS = 299792458.0;
  constexpr double kMicrosToSeconds = 1.0e-6;
  constexpr double kNanometersToMeters = 1.0e-9;

  // I(theta)=(m+1)P/(2*pi)*cos(theta)^m [W/sr]. `angular` is normalized
  // against the on-axis m=1 value, P/pi.
  const double radiant_intensity_w_sr = (power_w / M_PI) * angular;
  const double irradiance_w_m2 = radiant_intensity_w_sr /
      (distance_m * distance_m);
  const double aperture_area_m2 = M_PI * response.aperture_diameter_m
      * response.aperture_diameter_m / 4.0;
  const double collected_energy_j = irradiance_w_m2 * aperture_area_m2
      * response.optical_transmission
      * response.exposure_us * kMicrosToSeconds;
  const double photon_energy_j = kPlanckConstantJs * kSpeedOfLightMPerS /
      (response.wavelength_nm * kNanometersToMeters);
  const double photoelectrons = collected_energy_j / photon_energy_j
      * response.quantum_efficiency;
  return photoelectrons / response.electrons_per_adu * gain;
}

inline double gaussianPixelMass(
    double pixel_center_x,
    double pixel_center_y,
    double source_x,
    double source_y,
    double sigma_px)
{
  if (!std::isfinite(sigma_px) || sigma_px <= 0.0) {
    return 0.0;
  }

  const double scale = std::sqrt(2.0) * sigma_px;
  const auto intervalMass = [scale](double lo, double hi, double mean) {
    return 0.5 * (std::erf((hi - mean) / scale)
        - std::erf((lo - mean) / scale));
  };

  const double mass_x = intervalMass(
      pixel_center_x - 0.5, pixel_center_x + 0.5, source_x);
  const double mass_y = intervalMass(
      pixel_center_y - 0.5, pixel_center_y + 0.5, source_y);
  return std::max(0.0, mass_x * mass_y);
}

inline int psfSupportRadius(double signal_adu, double sigma_px)
{
  if (!std::isfinite(signal_adu) || signal_adu <= 0.0
      || !std::isfinite(sigma_px) || sigma_px <= 0.0) {
    return 0;
  }

  // Include the PSF until an approximate point sample contributes less than
  // a quarter of one output code.  The cap guards pathological near ranges.
  constexpr double kMinimumContributionAdu = 0.25;
  constexpr int kMaximumRadiusPx = 16;
  const double log_argument = std::max(1.0, signal_adu / kMinimumContributionAdu);
  const double radius = 0.5 + sigma_px * std::sqrt(2.0 * std::log(log_argument));
  return std::clamp(static_cast<int>(std::ceil(radius)), 1, kMaximumRadiusPx);
}

}  // namespace optics
}  // namespace uvdar_gazebo_plugin
