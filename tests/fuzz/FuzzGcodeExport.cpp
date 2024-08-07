// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Application.h"                // To set up a slice with settings.
#include "RetractionConfig.h"            // For extruder switch tests.
#include "Slice.h"                       // To set up a slice with settings.
#include "WipeScriptConfig.h"            // For wipe script tests.
#include "communication/Communication.h" //The interface we're implementing.
#include "gcodeExport.h"                 // The unit under test.
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"
#include "utils/Date.h" // To check the Griffin header.
#include <cmath>
#include <fuzzer/FuzzedDataProvider.h> // To create structured fuzz data.
#include <limits>
#include <string>

namespace cura {

class FuzzedCommunication : public Communication {
public:
  constexpr explicit FuzzedCommunication(FuzzedDataProvider *fdp) : fdp_(fdp) {}

  [[nodiscard]] bool hasSlice() const override { return fdp_->ConsumeBool(); }
  [[nodiscard]] bool isSequential() const override { return fdp_->ConsumeBool(); }
  void sendProgress(const float &progress) const override{};
  void sendLayerComplete(const LayerIndex &layer_nr, const coord_t &z,
                         const coord_t &thickness) override{};
   void sendPolygons(const PrintFeatureType &type,
                            const Polygons &polygons, const coord_t &line_width,
                            const coord_t &line_thickness,
                            const Velocity &velocity) override {}
   void sendPolygon(const PrintFeatureType &type,
                           const ConstPolygonRef &polygon,
                           const coord_t &line_width,
                           const coord_t &line_thickness,
                           const Velocity &velocity) override {}
   void sendLineTo(const PrintFeatureType &type, const Point &to,
                          const coord_t &line_width,
                          const coord_t &line_thickness,
                          const Velocity &velocity) override {}
   void sendCurrentPosition(const Point &position) override {}
   void setExtruderForSend(const ExtruderTrain &extruder) override {}
   void setLayerForSend(const LayerIndex &layer_nr) override {}
   void sendOptimizedLayerData() override {}
   void sendPrintTimeMaterialEstimates() const override {};
   void beginGCode() override {}
   void flushGCode() override {}
   void sendGCodePrefix(const std::string &prefix) const override {}
   void sendSliceUUID(const std::string &slice_uuid) const override {}
   void sendFinishedSlicing() const override {}
   void sliceNext() override {}

private:
  FuzzedDataProvider *fdp_;
};

enum GcodeExporterFunction {
  kSetSliceUUID,
  kSetLayerNumber,
  kSetFlavor,
  kSetZ,
  kSetFlowRateExtrusionSettings,
  kSetFilamentDiameter,
  kResetTotalPrintTimeAndFilament,
  kWriteComment,
  kWriteTypeComment,
  kWriteExtrusionMode,
  kResetExtrusionMode,
  kWriteTimeComment,
  kWriteLayerComment,
  kWriteLayerCountComment,
  kWriteLine,
  kResetExtrusionValue,
  kWriteDelay,
  kWriteTravel,
  kWriteExtrusion,
  kInitializeExtruderTrain,
  kProcessInitialLayerTemperature,
  kMaxValue = kProcessInitialLayerTemperature,
};

int initSettings(FuzzedDataProvider *fdp) {
  double layer_height = std::abs(fdp->ConsumeFloatingPoint<double>());
  if (!std::isfinite(layer_height)) {
    return 1;
  }
  Application::getInstance()
      .current_slice->scene.current_mesh_group->settings.add(
          "layer_height", std::to_string(layer_height));
  int number_of_extruders = fdp->ConsumeIntegralInRange<int>(1, MAX_EXTRUDERS);
  for (int i = 0; i < number_of_extruders; i++) {
    Scene &scene = Application::getInstance().current_slice->scene;
    scene.extruders.emplace_back(
        i, &Application::getInstance()
                .current_slice->scene.current_mesh_group->settings);
    ExtruderTrain &train = scene.extruders.back();
    train.settings.add(
        "machine_nozzle_size",
        std::to_string(std::abs(fdp->ConsumeFloatingPoint<double>())));
    train.settings.add("machine_nozzle_id", "TestNozzle-" + std::to_string(i));
    train.settings.add("machine_firmware_retract", "false");
  }
  return 0;
}

int fuzzGcodeExporter(FuzzedDataProvider *fdp) {

  std::stringstream output;
  GCodeExport gcode;
  gcode.setOutputStream(&output);
  int max_iterations = fdp->ConsumeIntegralInRange<int>(1, 2048);

  // Extruders must have a defined non-zero diameter to avoid devide by zeros.
  for (int i = 0; i < MAX_EXTRUDERS; i++) {
    gcode.setFilamentDiameter(0, 1);
  }
  for (int i = 0; i < max_iterations; i++) {
    if (fdp->remaining_bytes() == 0) {
      return 0;
    }
    switch (fdp->ConsumeEnum<GcodeExporterFunction>()) {
    case kSetSliceUUID: {
      constexpr int kUUIDLength = 32;
      gcode.setSliceUUID(fdp->ConsumeRandomLengthString(kUUIDLength));
      break;
    }
    case kSetLayerNumber:
      gcode.setLayerNr(fdp->ConsumeIntegral<unsigned int>());
      break;
    case kSetFlavor:
      gcode.setFlavor(fdp->ConsumeEnum<EGCodeFlavor>());
      break;
    case kSetZ:
      gcode.setZ(fdp->ConsumeIntegral<int>());
      break;
    case kSetFlowRateExtrusionSettings:
      gcode.setFlowRateExtrusionSettings(fdp->ConsumeFloatingPoint<double>(),
                                         fdp->ConsumeFloatingPoint<double>());
      break;
    case kSetFilamentDiameter:
      gcode.setFilamentDiameter(
          fdp->ConsumeIntegralInRange<size_t>(0, MAX_EXTRUDERS - 1),
          fdp->ConsumeIntegralInRange<coord_t>(
              1, std::numeric_limits<coord_t>::max()));
      break;
    case kResetTotalPrintTimeAndFilament:
      gcode.resetTotalPrintTimeAndFilament();
      break;
    case kWriteComment:
      gcode.writeComment(fdp->ConsumeRandomLengthString(256));
      break;
    case kWriteTypeComment:
      gcode.writeTypeComment(fdp->ConsumeEnum<PrintFeatureType>());
      break;
    case kWriteExtrusionMode:
      gcode.writeExtrusionMode(fdp->ConsumeBool());
      break;
    case kResetExtrusionMode:
      gcode.resetExtrusionMode();
      break;
    case kWriteTimeComment:
      gcode.writeTimeComment(std::abs(fdp->ConsumeFloatingPoint<double>()));
      break;
    case kWriteLayerComment:
      gcode.writeLayerComment(
          fdp->ConsumeIntegralInRange(0, std::numeric_limits<int>::max()));
      break;
    case kWriteLayerCountComment:
      gcode.writeLayerCountComment(
          fdp->ConsumeIntegralInRange(0, std::numeric_limits<int>::max()));
      break;
    case kWriteLine:
      gcode.writeLine(fdp->ConsumeRandomLengthString(256).c_str());
      break;
    case kResetExtrusionValue:
      gcode.resetExtrusionValue();
      break;
    case kWriteDelay:
      gcode.writeDelay(std::abs(fdp->ConsumeFloatingPoint<double>()));
      break;
    case kWriteTravel: {
      Point3 current_position = gcode.getPosition();
      // Total travel distance can't be > 1000.
      const int max_translation = 9;
      Point3 translation = Point3(
          fdp->ConsumeIntegralInRange<coord_t>(MM2INT(-max_translation / 2),
                                               MM2INT(max_translation / 2)),
          fdp->ConsumeIntegralInRange<coord_t>(MM2INT(-max_translation / 2),
                                               MM2INT(max_translation / 2)),
          fdp->ConsumeIntegralInRange<coord_t>(MM2INT(-max_translation / 2),
                                               MM2INT(max_translation / 2)));
      gcode.writeTravel(
          current_position + translation,
          Velocity(
              std::abs(fdp->ConsumeFloatingPointInRange<double>(1.1, 999.9))));
      break;
    }
    case kWriteExtrusion: {
      Point3 current_position = gcode.getPosition();
      // Total travel distance can't be > 1000.
      const int max_translation = 9;
      Point3 translation = Point3(
          fdp->ConsumeIntegralInRange<coord_t>(MM2INT(-max_translation / 2),
                                               MM2INT(max_translation / 2)),
          fdp->ConsumeIntegralInRange<coord_t>(MM2INT(-max_translation / 2),
                                               MM2INT(max_translation / 2)),
          fdp->ConsumeIntegralInRange<coord_t>(MM2INT(-max_translation / 2),
                                               MM2INT(max_translation / 2)));
      constexpr double max_extrusion_rate = 1000.0;
      gcode.writeExtrusion(
          current_position + translation,
          Velocity(
              std::abs(fdp->ConsumeFloatingPointInRange<double>(1.1, 999.9))),
          fdp->ConsumeFloatingPointInRange<double>(0.0, max_extrusion_rate),
          fdp->ConsumeEnum<PrintFeatureType>(), fdp->ConsumeBool());
      break;
    }
    case kInitializeExtruderTrain:
      // TODO: Implement a fuzzed version of the storage arg.
      break;
    case kProcessInitialLayerTemperature:
      // TODO: Implement a fuzzed version of the storage arg.
      break;
    }
  }
  return 0;
}

class App {
public:
  explicit App(FuzzedDataProvider *fdp) {
    Application::getInstance().current_slice = new Slice(1);
    Application::getInstance().communication = new FuzzedCommunication(fdp);
  }
  ~App() {
    delete Application::getInstance().current_slice;
    delete Application::getInstance().communication;
    Application::getInstance().communication = nullptr;
  }
};

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  FuzzedDataProvider fdp(data, size);
  App app(&fdp);
  if (initSettings(&fdp) != 0) {
    return 1;
  }
  int result = fuzzGcodeExporter(&fdp);

  return result;
}

} // namespace cura