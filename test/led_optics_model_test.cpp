#include <uvdar_gazebo_plugin/led_optics_model.hpp>

#include <cmath>
#include <iostream>

namespace
{

bool near(double actual, double expected, double tolerance)
{
  if (std::abs(actual - expected) <= tolerance) {
    return true;
  }
  std::cerr << "Expected " << expected << " +/- " << tolerance
            << ", got " << actual << std::endl;
  return false;
}

}  // namespace

int main()
{
  using uvdar_gazebo_plugin::optics::CameraResponse;
  using uvdar_gazebo_plugin::optics::gaussianPixelMass;
  using uvdar_gazebo_plugin::optics::integratedSignalAdu;
  using uvdar_gazebo_plugin::optics::lambertianRelativeIntensity;

  CameraResponse response;
  bool ok = true;

  const double near_signal = integratedSignalAdu(
      response, 1.0, 1.0, 1.0, 2.0);
  const double far_signal = integratedSignalAdu(
      response, 1.0, 1.0, 1.0, 4.0);
  ok &= near(near_signal / far_signal, 4.0, 1.0e-12);
  ok &= near(integratedSignalAdu(response, 0.5, 1.0, 1.0, 2.0),
      0.5 * near_signal, 1.0e-9);
  ok &= near(integratedSignalAdu(response, 1.0, 0.5, 1.0, 2.0),
      0.5 * near_signal, 1.0e-9);
  ok &= near(lambertianRelativeIntensity(1.0, 2.0), 1.5, 1.0e-12);
  ok &= near(lambertianRelativeIntensity(-0.1, 1.0), 0.0, 1.0e-12);

  CameraResponse longer_exposure = response;
  longer_exposure.exposure_us *= 2.0;
  ok &= near(integratedSignalAdu(
      longer_exposure, 1.0, 1.0, 1.0, 2.0), 2.0 * near_signal, 1.0e-9);

  CameraResponse larger_aperture = response;
  larger_aperture.aperture_diameter_m *= 2.0;
  ok &= near(integratedSignalAdu(
      larger_aperture, 1.0, 1.0, 1.0, 2.0), 4.0 * near_signal, 1.0e-9);

  CameraResponse lower_throughput = response;
  lower_throughput.optical_transmission *= 0.5;
  ok &= near(integratedSignalAdu(
      lower_throughput, 1.0, 1.0, 1.0, 2.0), 0.5 * near_signal, 1.0e-9);

  CameraResponse lower_efficiency = response;
  lower_efficiency.quantum_efficiency *= 0.5;
  ok &= near(integratedSignalAdu(
      lower_efficiency, 1.0, 1.0, 1.0, 2.0), 0.5 * near_signal, 1.0e-9);

  CameraResponse lower_conversion_gain = response;
  lower_conversion_gain.electrons_per_adu *= 2.0;
  ok &= near(integratedSignalAdu(
      lower_conversion_gain, 1.0, 1.0, 1.0, 2.0), 0.5 * near_signal, 1.0e-9);

  // There is no range cutoff: signal remains positive and decays continuously.
  if (integratedSignalAdu(response, 1.0, 1.0, 1.0, 1.0e6) <= 0.0) {
    std::cerr << "Radiometric response unexpectedly has a range cutoff" << std::endl;
    ok = false;
  }

  double psf_mass = 0.0;
  for (int y = -6; y <= 6; ++y) {
    for (int x = -6; x <= 6; ++x) {
      psf_mass += gaussianPixelMass(
          static_cast<double>(x), static_cast<double>(y), 0.0, 0.0,
          response.psf_sigma_px);
    }
  }
  ok &= near(psf_mass, 1.0, 1.0e-12);

  const double point_signal = integratedSignalAdu(
      response, 1.0, 1.0, 1.0, 4.0);
  const double center_signal = point_signal
      * gaussianPixelMass(0.0, 0.0, 0.0, 0.0, response.psf_sigma_px);
  const double adjacent_signal = point_signal
      * gaussianPixelMass(1.0, 0.0, 0.0, 0.0, response.psf_sigma_px);
  if (center_signal <= adjacent_signal) {
    std::cerr << "The PSF centre is not brighter than its neighbour" << std::endl;
    ok = false;
  }

  // At an exact four-pixel boundary, the four closest pixels receive equal
  // energy, preserving the projected subpixel centre.
  const double boundary_mass = gaussianPixelMass(
      0.0, 0.0, 0.5, 0.5, response.psf_sigma_px);
  ok &= near(boundary_mass, gaussianPixelMass(
      1.0, 1.0, 0.5, 0.5, response.psf_sigma_px), 1.0e-12);

  return ok ? 0 : 1;
}
