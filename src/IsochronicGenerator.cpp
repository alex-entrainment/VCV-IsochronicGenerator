#include "plugin.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr float kDefaultSampleRate = 44100.f;
constexpr float kMinFrequencyHz = 1.f;
constexpr float kMaxFrequencyHz = 20000.f;
constexpr float kAudioLevelVolts = 5.f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr int kSchemaVersion = 2;
constexpr int kMaxSupportedPolyphony = 4;

constexpr std::array<float, 4> kBeatMaxOptions = {100.f, 50.f, 20.f, 10.f};
constexpr std::array<float, 5> kCarrierMaxOptions = {20000.f, 10000.f, 5000.f,
                                                     2000.f, 1000.f};
constexpr std::array<float, 4> kAmRateMaxOptions = {100.f, 50.f, 20.f, 10.f};
constexpr std::array<float, 7> kFmRateMaxOptions = {100.f, 50.f, 20.f, 10.f,
                                                    5.f,   2.f,  1.f};
constexpr std::array<float, 4> kFmRangeMaxOptions = {100.f, 50.f, 20.f, 10.f};
constexpr std::array<float, 2> kVOctMinOptions = {-5.f, 0.f};
constexpr std::array<float, 2> kVOctMaxOptions = {5.f, 10.f};
constexpr std::array<int, 4> kPolyphonyChannelOptions = {1, 2, 3, 4};

struct EffectiveControls {
  float beatHz = 4.f;
  float carrierHz = 261.63f;
  float amRateHz = 0.f;
  float amDepth = 0.f;
  float fmRateHz = 0.f;
  float fmRangeHz = 0.f;
  bool leftHigh = false;
};

struct PersistentState {
  int schemaVersion = kSchemaVersion;
  int beatRangeIndex = 0;
  int carrierRangeIndex = 0;
  int amRateRangeIndex = 0;
  int fmRateRangeIndex = 0;
  int fmRangeRangeIndex = 0;
  int vOctRangeIndex = 0;
  int polyphonyModeIndex = 0;
};

template <size_t N> size_t clampIndex(int index) {
  if (index < 0) {
    return 0;
  }
  const size_t converted = static_cast<size_t>(index);
  return std::min(converted, N - 1);
}

float wrapPhase(float phase) {
  phase -= std::floor(phase);
  if (phase < 0.f) {
    phase += 1.f;
  }
  return phase;
}

float clampFrequency(float frequencyHz) {
  return clamp(frequencyHz, kMinFrequencyHz, kMaxFrequencyHz);
}

float applyScaledCv(float knobValue, float cvVoltage, float cvAmount,
                    float minValue, float maxValue) {
  const float span = maxValue - minValue;
  const float normalizedCv = clamp(cvVoltage / 5.f, -1.f, 1.f);
  return clamp(knobValue + normalizedCv * cvAmount * span, minValue, maxValue);
}

}

struct IsochronicGenerator : Module {
  enum ParamId {
    ALTERNATE_MODE_SWITCH_PARAM,
    BF_CV_KNOB_PARAM,
    BF_KNOB_PARAM,
    CF_KNOB_PARAM,
    AM_RATE_KNOB_PARAM,
    AM_DEPTH_KNOB_PARAM,
    FM_RATE_KNOB_PARAM,
    FM_RANGE_KNOB_PARAM,
    AM_RATE_CV_KNOB_PARAM,
    AM_DEPTH_CV_KNOB_PARAM,
    FM_RATE_CV_KNOB_PARAM,
    FM_RANGE_CV_KNOB_PARAM,
    GAP_P_KNOB_PARAM,
    RAMP_P_CV_KNOB_PARAM,
    RAMP_P_KNOB_PARAM,
    GAP_P_CV_KNOB_PARAM,
    PARAMS_LEN
  };
  enum InputId {
    V_OCT_IN_INPUT, // TODO: FIX V_OCT_PARAM INSTANCES -> INPUT
    BF_CV_IN_INPUT,
    AMR_CV_IN_INPUT,
    AMD_CV_IN_INPUT,
    FMRATE_CV_IN_INPUT,
    FMRANGE_CV_IN_INPUT,
    RAMP_CV_IN_INPUT,
    GAP_CV_IN_INPUT,
    INPUTS_LEN
  };
  enum OutputId { R_OUT_OUTPUT, L_OUT_OUTPUT, OUTPUTS_LEN };
  enum LightId { BF_LIGHT_LIGHT, LIGHTS_LEN };

  float sampleRate = kDefaultSampleRate;
  float sampleTime = 1.f / kDefaultSampleRate;
  float smoothingFactor = 1.f;
  float beatLightPhase = 0.f;
  std::array<float, kMaxSupportedPolyphony> leftPhases = {};
  std::array<float, kMaxSupportedPolyphony> rightPhases = {};
  std::array<float, kMaxSupportedPolyphony> amPhases = {};
  std::array<float, kMaxSupportedPolyphony> fmPhases = {};
  std::array<bool, kMaxSupportedPolyphony> controlsPrimed = {};
  std::array<EffectiveControls, kMaxSupportedPolyphony> smoothedControls = {};
  PersistentState persistentState;

  IsochronicGenerator() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    configParam(BF_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "Beat frequency CV %", "%",
                0.f, 100.f);
    configParam(BF_KNOB_PARAM, 0.f, kBeatMaxOptions.front(), 4.f,
                "Beat frequency", " Hz");
    configParam(CF_KNOB_PARAM, 1.f, kCarrierMaxOptions.front(), 261.63f,
                "Carrier frequency", " Hz");
    configParam(AM_RATE_KNOB_PARAM, 0.f, kAmRateMaxOptions.front(), 0.f,
                "Amplitude modulation rate", " Hz");
    configParam(AM_DEPTH_KNOB_PARAM, 0.f, 1.f, 0.f,
                "Amplitude modulation depth");
    configParam(FM_RATE_KNOB_PARAM, 0.f, kFmRateMaxOptions.front(), 0.f,
                "Frequency modulation rate", " Hz");
    configParam(FM_RANGE_KNOB_PARAM, 0.f, kFmRangeMaxOptions.front(), 0.f,
                "Frequency modulation range", " Hz");
    configParam(AM_RATE_CV_KNOB_PARAM, -1.f, 1.f, 0.f,
                "Amplitude modulation rate CV %", "%", 0.f, 100.f);
    configParam(AM_DEPTH_CV_KNOB_PARAM, -1.f, 1.f, 0.f,
                "Amplitude modulation depth CV %", "%", 0.f, 100.f);
    configParam(FM_RATE_CV_KNOB_PARAM, -1.f, 1.f, 0.f,
                "Frequency modulation rate CV %", "%", 0.f, 100.f);
    configParam(FM_RANGE_CV_KNOB_PARAM, -1.f, 1.f, 0.f,
                "Frequency modulation range CV %", "%", 0.f, 100.f);
    configParam(GAP_P_KNOB_PARAM, 0.f, 1.f, 0.f, // Adjusts % of "gap time" in isochronics 
                "Tone Gap %", "%", 0.f, 100.f);
    configParam(RAMP_P_CV_KNOB_PARAM, 0.f, 1.f, 0.f,  // Adjusts % of "ramp time" in isochronics
                "Tone Ramp %", "%", 0.f, 100.f);
    configParam(RAMP_P_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(GAP_P_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configSwitch(ALTERNATE_MODE_SWITCH_PARAM, 0.f, 1.f, 0.f, {"Synced", "Alternating"});

    configInput(V_OCT_IN_INPUT, "Pitch CV"); // V/Oct Pitch input 
    configInput(BF_CV_IN_INPUT, "Beat Freq CV");                
    configInput(AMR_CV_IN_INPUT, "AM Rate CV");
    configInput(AMD_CV_IN_INPUT, "AM Depth CV");
    configInput(FMRATE_CV_IN_INPUT, "FM Rate CV");
    configInput(FMRANGE_CV_IN_INPUT, "FM Range CV");
    configInput(RAMP_CV_IN_INPUT, "Ramp % CV");
    configInput(GAP_CV_IN_INPUT, " Gap % CV");
    configOutput(R_OUT_OUTPUT, "L Out");
    configOutput(L_OUT_OUTPUT, "R Out");

    refreshSampleRate();
    applyPersistentState();
    resetRuntimeState();
  }

  void process(const ProcessArgs &args) override {}
};

struct IsochronicGeneratorWidget : ModuleWidget {
  IsochronicGeneratorWidget(IsochronicGenerator *module) {
    setModule(module);
    setPanel(createPanel(
        asset::plugin(pluginInstance, "res/IsochronicGenerator.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(
        createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(
        Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(
        box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(257.282, 35.2)), module,
        IsochronicGenerator::ALTERNATE_MODE_SWITCH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(81.547, 92.438)), module,
        IsochronicGenerator::BF_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(150.337, 92.438)), module,
        IsochronicGenerator::BF_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(257.282, 92.438)), module,
        IsochronicGenerator::CF_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(43.664, 177.908)), module,
        IsochronicGenerator::AM_RATE_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(114.559, 177.908)), module,
        IsochronicGenerator::AM_DEPTH_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(186.04, 177.908)), module,
        IsochronicGenerator::FM_RATE_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(257.01, 177.908)), module,
        IsochronicGenerator::FM_RANGE_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(43.664, 235.01)), module,
        IsochronicGenerator::AM_RATE_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(114.559, 235.01)), module,
        IsochronicGenerator::AM_DEPTH_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(186.754, 235.01)), module,
        IsochronicGenerator::FM_RATE_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(257.01, 235.01)), module,
        IsochronicGenerator::FM_RANGE_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(43.664, 313.104)), module,
        IsochronicGenerator::GAP_P_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(114.559, 313.104)), module,
        IsochronicGenerator::RAMP_P_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(43.664, 347.418)), module,
        IsochronicGenerator::RAMP_P_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        mm2px(Vec(114.559, 347.418)), module,
        IsochronicGenerator::GAP_P_CV_KNOB_PARAM));

    addInput(createInputCentered<PJ301MPort>(
        mm2px(Vec(27.507, 35.2)), module, IsochronicGenerator::V_OCT_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(mm2px(Vec(27.507, 92.032)), module,
                                        IsochronicGenerator::BF_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(mm2px(Vec(43.664, 277.822)), module,
                                        IsochronicGenerator::AMR_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(mm2px(Vec(114.559, 277.822)), module,
                                        IsochronicGenerator::AMD_CV_IN_INPUT));
    addInput(createInputCentered<PJ301MPort>(
        mm2px(Vec(185.758, 277.822)), module,
        IsochronicGenerator::FMRATE_CV_IN_INPUT));
    addInput(createInputCentered<PJ301MPort>(
        mm2px(Vec(257.01, 277.822)), module,
        IsochronicGenerator::FMRANGE_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(mm2px(Vec(185.758, 314.501)), module,
                                        IsochronicGenerator::RAMP_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(mm2px(Vec(185.758, 347.418)), module,
                                        IsochronicGenerator::GAP_CV_IN_INPUT));

    addOutput(
        createOutputCentered<PJ301MPort>(mm2px(Vec(256.014, 313.104)), module,
                                         IsochronicGenerator::R_OUT_OUTPUT));
    addOutput(
        createOutputCentered<PJ301MPort>(mm2px(Vec(256.014, 347.418)), module,
                                         IsochronicGenerator::L_OUT_OUTPUT));

    addChild(createLightCentered<MediumLight<RedLight>>(
        mm2px(Vec(183.061, 92.438)), module,
        IsochronicGenerator::BF_LIGHT_LIGHT));
  }
};

Model *modelIsochronicGenerator =
    createModel<IsochronicGenerator, IsochronicGeneratorWidget>(
        "IsochronicGenerator");
