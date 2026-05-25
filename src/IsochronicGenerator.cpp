#include "plugin.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr float kDefaultSampleRate = 44100.f;
constexpr float kMinFrequencyHz = 1.f;
constexpr float kMaxFrequencyHz = 20000.f;
constexpr float kAudioLevelVolts = 5.f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kControlSmoothingSeconds = 0.01f;
constexpr float kBeatStoppedThresholdHz = 0.001f;
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
  float gapFraction = 0.f;
  float rampFraction = 0.f;
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

void clampEnvelopeControls(float &gapFraction, float &rampFraction) {
  gapFraction = clamp(gapFraction, 0.f, 1.f);
  rampFraction = clamp(rampFraction, 0.f, 1.f);
}

float computeIsochronicEnvelope(float phase, float gapFraction,
                                float rampFraction) {
  clampEnvelopeControls(gapFraction, rampFraction);

  const float audibleFraction = 1.f - gapFraction;
  if (audibleFraction <= 0.f) {
    return 0.f;
  }

  float rampTotalFraction = audibleFraction * rampFraction * 2.f;
  rampTotalFraction = std::min(rampTotalFraction, audibleFraction);

  const float stableFraction = audibleFraction - rampTotalFraction;
  const float rampUpFraction = rampTotalFraction * 0.5f;
  const float stableEnd = rampUpFraction + stableFraction;

  if (phase >= audibleFraction) {
    return 0.f;
  }
  if (phase < rampUpFraction) {
    return (rampUpFraction <= 0.f) ? 0.f : phase / rampUpFraction;
  }
  if (phase < stableEnd) {
    return 1.f;
  }
  if (rampUpFraction <= 0.f) {
    return 0.f;
  }
  return 1.f - (phase - stableEnd) / rampUpFraction;
}

} // namespace

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
    V_OCT_IN_INPUT,
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
  float lightEnvelope = 0.f;
  std::array<float, kMaxSupportedPolyphony> leftPhases = {};
  std::array<float, kMaxSupportedPolyphony> rightPhases = {};
  std::array<float, kMaxSupportedPolyphony> beatPhases = {};
  std::array<bool, kMaxSupportedPolyphony> alternatingPulseOnLeft = {};
  std::array<float, kMaxSupportedPolyphony> amPhases = {};
  std::array<float, kMaxSupportedPolyphony> fmPhases = {};
  std::array<bool, kMaxSupportedPolyphony> controlsPrimed = {};
  std::array<EffectiveControls, kMaxSupportedPolyphony> smoothedControls = {};
  PersistentState persistentState;

  IsochronicGenerator() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configSwitch(ALTERNATE_MODE_SWITCH_PARAM, 0.f, 1.f, 0.f, "Output mode",
                 {"Synced", "Alternating"});
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
    configParam(GAP_P_KNOB_PARAM, 0.f, 1.f, 0.15f, "Tone gap %", "%", 0.f,
                100.f);
    configParam(RAMP_P_CV_KNOB_PARAM, -1.f, 1.f, 0.f, "Tone ramp CV %", "%",
                0.f, 100.f);
    configParam(RAMP_P_KNOB_PARAM, 0.f, 1.f, 0.2f, "Tone ramp %", "%", 0.f,
                100.f);
    configParam(GAP_P_CV_KNOB_PARAM, -1.f, 1.f, 0.f, "Tone gap CV %", "%",
                0.f, 100.f);

    configInput(V_OCT_IN_INPUT, "Pitch CV");
    configInput(BF_CV_IN_INPUT, "Beat frequency CV");
    configInput(AMR_CV_IN_INPUT, "Amplitude modulation rate CV");
    configInput(AMD_CV_IN_INPUT, "Amplitude modulation depth CV");
    configInput(FMRATE_CV_IN_INPUT, "Frequency modulation rate CV");
    configInput(FMRANGE_CV_IN_INPUT, "Frequency modulation range CV");
    configInput(RAMP_CV_IN_INPUT, "Ramp % CV");
    configInput(GAP_CV_IN_INPUT, "Gap % CV");

    configOutput(L_OUT_OUTPUT, "Left output");
    configOutput(R_OUT_OUTPUT, "Right output");

    refreshSampleRate();
    applyPersistentState();
    resetRuntimeState();
  }

  void refreshSampleRate() {
    if (APP && APP->engine) {
      sampleRate = APP->engine->getSampleRate();
    }
    if (!(sampleRate > 0.f)) {
      sampleRate = kDefaultSampleRate;
    }
    sampleTime = 1.f / sampleRate;
    smoothingFactor = std::exp(-sampleTime / kControlSmoothingSeconds);
  }

  void resetSmoothedControlState() {
    controlsPrimed.fill(false);
    smoothedControls.fill(EffectiveControls());
  }

  void resetRuntimeState() {
    leftPhases.fill(0.f);
    rightPhases.fill(0.f);
    beatPhases.fill(0.f);
    alternatingPulseOnLeft.fill(true);
    amPhases.fill(0.f);
    fmPhases.fill(0.f);
    lightEnvelope = 0.f;
    resetSmoothedControlState();
  }

  void applyParamLimit(int paramId, float minValue, float maxValue) {
    if (engine::ParamQuantity *quantity = getParamQuantity(paramId)) {
      quantity->minValue = minValue;
      quantity->maxValue = maxValue;
      quantity->defaultValue = clamp(quantity->defaultValue, minValue, maxValue);
      quantity->setValue(clamp(quantity->getValue(), minValue, maxValue));
    }
  }

  void applyPersistentState() {
    persistentState.schemaVersion = kSchemaVersion;
    persistentState.beatRangeIndex =
        static_cast<int>(clampIndex<kBeatMaxOptions.size()>(
            persistentState.beatRangeIndex));
    persistentState.carrierRangeIndex =
        static_cast<int>(clampIndex<kCarrierMaxOptions.size()>(
            persistentState.carrierRangeIndex));
    persistentState.amRateRangeIndex =
        static_cast<int>(clampIndex<kAmRateMaxOptions.size()>(
            persistentState.amRateRangeIndex));
    persistentState.fmRateRangeIndex =
        static_cast<int>(clampIndex<kFmRateMaxOptions.size()>(
            persistentState.fmRateRangeIndex));
    persistentState.fmRangeRangeIndex =
        static_cast<int>(clampIndex<kFmRangeMaxOptions.size()>(
            persistentState.fmRangeRangeIndex));
    persistentState.vOctRangeIndex =
        static_cast<int>(clampIndex<kVOctMinOptions.size()>(
            persistentState.vOctRangeIndex));
    persistentState.polyphonyModeIndex =
        static_cast<int>(clampIndex<kPolyphonyChannelOptions.size()>(
            persistentState.polyphonyModeIndex));

    applyParamLimit(BF_KNOB_PARAM, 0.f,
                    kBeatMaxOptions[persistentState.beatRangeIndex]);
    applyParamLimit(CF_KNOB_PARAM, 1.f,
                    kCarrierMaxOptions[persistentState.carrierRangeIndex]);
    applyParamLimit(AM_RATE_KNOB_PARAM, 0.f,
                    kAmRateMaxOptions[persistentState.amRateRangeIndex]);
    applyParamLimit(FM_RATE_KNOB_PARAM, 0.f,
                    kFmRateMaxOptions[persistentState.fmRateRangeIndex]);
    applyParamLimit(FM_RANGE_KNOB_PARAM, 0.f,
                    kFmRangeMaxOptions[persistentState.fmRangeRangeIndex]);

    resetSmoothedControlState();
  }

  int getSelectedPolyphonyChannels() {
    const size_t index =
        clampIndex<kPolyphonyChannelOptions.size()>(
            persistentState.polyphonyModeIndex);
    return kPolyphonyChannelOptions[index];
  }

  int getActiveChannels() {
    const int selectedChannels = getSelectedPolyphonyChannels();
    const int pitchChannels = inputs[V_OCT_IN_INPUT].getChannels();
    if (pitchChannels > 1) {
      return std::min(selectedChannels, pitchChannels);
    }
    return selectedChannels;
  }

  float getInputVoltageForChannel(int inputId, int channel) {
    return inputs[inputId].getNormalPolyVoltage(0.f, channel);
  }

  float getPitchVoltageForChannel(int channel) {
    const size_t rangeIndex =
        clampIndex<kVOctMinOptions.size()>(persistentState.vOctRangeIndex);
    const float voltage = getInputVoltageForChannel(V_OCT_IN_INPUT, channel);
    return clamp(voltage, kVOctMinOptions[rangeIndex], kVOctMaxOptions[rangeIndex]);
  }

  EffectiveControls resolveTargets(int channel) {
    EffectiveControls targets;
    const float beatMax = kBeatMaxOptions[clampIndex<kBeatMaxOptions.size()>(
        persistentState.beatRangeIndex)];
    const float carrierMax =
        kCarrierMaxOptions[clampIndex<kCarrierMaxOptions.size()>(
            persistentState.carrierRangeIndex)];
    const float amRateMax = kAmRateMaxOptions[clampIndex<kAmRateMaxOptions.size()>(
        persistentState.amRateRangeIndex)];
    const float fmRateMax = kFmRateMaxOptions[clampIndex<kFmRateMaxOptions.size()>(
        persistentState.fmRateRangeIndex)];
    const float fmRangeMax =
        kFmRangeMaxOptions[clampIndex<kFmRangeMaxOptions.size()>(
            persistentState.fmRangeRangeIndex)];

    targets.beatHz =
        applyScaledCv(params[BF_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(BF_CV_IN_INPUT, channel),
                      params[BF_CV_KNOB_PARAM].getValue(), 0.f, beatMax);

    const float pitchVolts = getPitchVoltageForChannel(channel);
    const float baseCarrierHz =
        clamp(params[CF_KNOB_PARAM].getValue(), 1.f, carrierMax) *
        std::pow(2.f, pitchVolts);
    targets.carrierHz = clampFrequency(baseCarrierHz);

    targets.amRateHz =
        applyScaledCv(params[AM_RATE_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(AMR_CV_IN_INPUT, channel),
                      params[AM_RATE_CV_KNOB_PARAM].getValue(), 0.f, amRateMax);
    targets.amDepth =
        applyScaledCv(params[AM_DEPTH_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(AMD_CV_IN_INPUT, channel),
                      params[AM_DEPTH_CV_KNOB_PARAM].getValue(), 0.f, 1.f);
    targets.fmRateHz =
        applyScaledCv(params[FM_RATE_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(FMRATE_CV_IN_INPUT, channel),
                      params[FM_RATE_CV_KNOB_PARAM].getValue(), 0.f, fmRateMax);
    targets.fmRangeHz =
        applyScaledCv(params[FM_RANGE_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(FMRANGE_CV_IN_INPUT, channel),
                      params[FM_RANGE_CV_KNOB_PARAM].getValue(), 0.f,
                      fmRangeMax);
    targets.gapFraction =
        applyScaledCv(params[GAP_P_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(GAP_CV_IN_INPUT, channel),
                      params[GAP_P_CV_KNOB_PARAM].getValue(), 0.f, 1.f);
    targets.rampFraction =
        applyScaledCv(params[RAMP_P_KNOB_PARAM].getValue(),
                      getInputVoltageForChannel(RAMP_CV_IN_INPUT, channel),
                      params[RAMP_P_CV_KNOB_PARAM].getValue(), 0.f, 1.f);

    clampEnvelopeControls(targets.gapFraction, targets.rampFraction);
    return targets;
  }

  void primeSmoothedControls(int channel, const EffectiveControls &targets) {
    smoothedControls[channel] = targets;
    controlsPrimed[channel] = true;
  }

  void smoothControls(int channel, const EffectiveControls &targets) {
    const float blend = 1.f - smoothingFactor;
    smoothedControls[channel].beatHz +=
        (targets.beatHz - smoothedControls[channel].beatHz) * blend;
    smoothedControls[channel].carrierHz +=
        (targets.carrierHz - smoothedControls[channel].carrierHz) * blend;
    smoothedControls[channel].amRateHz +=
        (targets.amRateHz - smoothedControls[channel].amRateHz) * blend;
    smoothedControls[channel].amDepth +=
        (targets.amDepth - smoothedControls[channel].amDepth) * blend;
    smoothedControls[channel].fmRateHz +=
        (targets.fmRateHz - smoothedControls[channel].fmRateHz) * blend;
    smoothedControls[channel].fmRangeHz +=
        (targets.fmRangeHz - smoothedControls[channel].fmRangeHz) * blend;
    smoothedControls[channel].gapFraction +=
        (targets.gapFraction - smoothedControls[channel].gapFraction) * blend;
    smoothedControls[channel].rampFraction +=
        (targets.rampFraction - smoothedControls[channel].rampFraction) * blend;

    clampEnvelopeControls(smoothedControls[channel].gapFraction,
                          smoothedControls[channel].rampFraction);
  }

  json_t *dataToJson() override {
    json_t *rootJ = json_object();
    json_object_set_new(rootJ, "schemaVersion",
                        json_integer(kSchemaVersion));
    json_object_set_new(rootJ, "beatRangeIndex",
                        json_integer(persistentState.beatRangeIndex));
    json_object_set_new(rootJ, "carrierRangeIndex",
                        json_integer(persistentState.carrierRangeIndex));
    json_object_set_new(rootJ, "amRateRangeIndex",
                        json_integer(persistentState.amRateRangeIndex));
    json_object_set_new(rootJ, "fmRateRangeIndex",
                        json_integer(persistentState.fmRateRangeIndex));
    json_object_set_new(rootJ, "fmRangeRangeIndex",
                        json_integer(persistentState.fmRangeRangeIndex));
    json_object_set_new(rootJ, "vOctRangeIndex",
                        json_integer(persistentState.vOctRangeIndex));
    json_object_set_new(rootJ, "polyphonyModeIndex",
                        json_integer(persistentState.polyphonyModeIndex));
    return rootJ;
  }

  void dataFromJson(json_t *rootJ) override {
    persistentState = PersistentState();

    if (!rootJ) {
      applyPersistentState();
      resetRuntimeState();
      return;
    }

    auto readIndex = [rootJ](const char *key, int fallback) {
      json_t *valueJ = json_object_get(rootJ, key);
      if (valueJ && json_is_integer(valueJ)) {
        return static_cast<int>(json_integer_value(valueJ));
      }
      return fallback;
    };

    persistentState.schemaVersion =
        readIndex("schemaVersion", persistentState.schemaVersion);
    persistentState.beatRangeIndex =
        readIndex("beatRangeIndex", persistentState.beatRangeIndex);
    persistentState.carrierRangeIndex =
        readIndex("carrierRangeIndex", persistentState.carrierRangeIndex);
    persistentState.amRateRangeIndex =
        readIndex("amRateRangeIndex", persistentState.amRateRangeIndex);
    persistentState.fmRateRangeIndex =
        readIndex("fmRateRangeIndex", persistentState.fmRateRangeIndex);
    persistentState.fmRangeRangeIndex =
        readIndex("fmRangeRangeIndex", persistentState.fmRangeRangeIndex);
    persistentState.vOctRangeIndex =
        readIndex("vOctRangeIndex", persistentState.vOctRangeIndex);
    persistentState.polyphonyModeIndex =
        readIndex("polyphonyModeIndex", persistentState.polyphonyModeIndex);

    applyPersistentState();
    resetRuntimeState();
  }

  void onReset(const ResetEvent &e) override {
    Module::onReset(e);
    applyPersistentState();
    resetRuntimeState();
  }

  void onSampleRateChange(const SampleRateChangeEvent &e) override {
    refreshSampleRate();
  }

  void process(const ProcessArgs &args) override {
    const int activeChannels = getActiveChannels();
    outputs[L_OUT_OUTPUT].setChannels(activeChannels);
    outputs[R_OUT_OUTPUT].setChannels(activeChannels);

    const bool alternatingMode =
        params[ALTERNATE_MODE_SWITCH_PARAM].getValue() >= 0.5f;
    float firstChannelEnvelope = 0.f;

    for (int channel = 0; channel < activeChannels; ++channel) {
      const EffectiveControls targets = resolveTargets(channel);
      if (!controlsPrimed[channel]) {
        primeSmoothedControls(channel, targets);
      }
      smoothControls(channel, targets);

      EffectiveControls controls = smoothedControls[channel];
      clampEnvelopeControls(controls.gapFraction, controls.rampFraction);

      fmPhases[channel] =
          wrapPhase(fmPhases[channel] + controls.fmRateHz * args.sampleTime);
      amPhases[channel] =
          wrapPhase(amPhases[channel] + controls.amRateHz * args.sampleTime);
      if (controls.beatHz > kBeatStoppedThresholdHz) {
        const float nextBeatPhase =
            beatPhases[channel] + controls.beatHz * args.sampleTime;
        if (nextBeatPhase >= 1.f && alternatingMode) {
          alternatingPulseOnLeft[channel] = !alternatingPulseOnLeft[channel];
        }
        beatPhases[channel] = wrapPhase(nextBeatPhase);
      }

      const float fmOffsetHz =
          std::sin(kTwoPi * fmPhases[channel]) * controls.fmRangeHz;
      const float instantaneousCarrierHz =
          clampFrequency(controls.carrierHz + fmOffsetHz);
      const float amGain =
          1.f - controls.amDepth * 0.5f *
                    (std::sin(kTwoPi * amPhases[channel]) + 1.f);

      leftPhases[channel] = wrapPhase(leftPhases[channel] +
                                      instantaneousCarrierHz * args.sampleTime);
      rightPhases[channel] = wrapPhase(rightPhases[channel] +
                                       instantaneousCarrierHz * args.sampleTime);

      float leftEnvelope = 1.f;
      float rightEnvelope = 1.f;
      if (controls.beatHz > kBeatStoppedThresholdHz) {
        const float baseEnvelope =
            computeIsochronicEnvelope(beatPhases[channel], controls.gapFraction,
                                      controls.rampFraction);

        leftEnvelope = baseEnvelope;
        rightEnvelope = baseEnvelope;
        if (alternatingMode) {
          const bool pulseOnLeft = alternatingPulseOnLeft[channel];
          leftEnvelope = pulseOnLeft ? baseEnvelope : 0.f;
          rightEnvelope = pulseOnLeft ? 0.f : baseEnvelope;
        }
      }

      const float leftCarrier = std::sin(kTwoPi * leftPhases[channel]);
      const float rightCarrier = alternatingMode
                                     ? std::sin(kTwoPi * rightPhases[channel])
                                     : leftCarrier;
      const float leftOutput =
          leftCarrier * amGain * leftEnvelope * kAudioLevelVolts;
      const float rightOutput =
          rightCarrier * amGain * rightEnvelope * kAudioLevelVolts;

      outputs[L_OUT_OUTPUT].setVoltage(leftOutput, channel);
      outputs[R_OUT_OUTPUT].setVoltage(rightOutput, channel);

      if (channel == 0) {
        firstChannelEnvelope = std::max(leftEnvelope, rightEnvelope);
      }
    }

    lightEnvelope += (firstChannelEnvelope - lightEnvelope) *
                     std::min(1.f, args.sampleTime * 30.f);
    lights[BF_LIGHT_LIGHT].setBrightnessSmooth(lightEnvelope, args.sampleTime);
  }
};

struct IsochronicGeneratorWidget : ModuleWidget {
  IsochronicGeneratorWidget(IsochronicGenerator *module) {
    setModule(module);
    setPanel(createPanel(
        asset::plugin(pluginInstance, "res/Isochronic Generator.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(
        createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(
        Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(
        box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParamCentered<CKSS>(Vec(257.282, 35.2), module,
                                       IsochronicGenerator::
                                           ALTERNATE_MODE_SWITCH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(81.547, 92.438), module,
        IsochronicGenerator::BF_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(150.337, 92.438), module,
        IsochronicGenerator::BF_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(257.282, 92.438), module,
        IsochronicGenerator::CF_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(43.664, 177.908), module,
        IsochronicGenerator::AM_RATE_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(114.559, 177.908), module,
        IsochronicGenerator::AM_DEPTH_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(186.04, 177.908), module,
        IsochronicGenerator::FM_RATE_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(257.01, 177.908), module,
        IsochronicGenerator::FM_RANGE_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(43.664, 235.01), module,
        IsochronicGenerator::AM_RATE_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(114.559, 235.01), module,
        IsochronicGenerator::AM_DEPTH_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(186.754, 235.01), module,
        IsochronicGenerator::FM_RATE_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(257.01, 235.01), module,
        IsochronicGenerator::FM_RANGE_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(43.664, 313.104), module,
        IsochronicGenerator::GAP_P_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(114.559, 313.104), module,
        IsochronicGenerator::RAMP_P_CV_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(43.664, 347.418), module,
        IsochronicGenerator::RAMP_P_KNOB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(
        Vec(114.559, 347.418), module,
        IsochronicGenerator::GAP_P_CV_KNOB_PARAM));

    addInput(createInputCentered<PJ301MPort>(
        Vec(27.507, 35.2), module, IsochronicGenerator::V_OCT_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(Vec(27.507, 92.032), module,
                                        IsochronicGenerator::BF_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(Vec(43.664, 277.822), module,
                                        IsochronicGenerator::AMR_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(Vec(114.559, 277.822), module,
                                        IsochronicGenerator::AMD_CV_IN_INPUT));
    addInput(createInputCentered<PJ301MPort>(
        Vec(185.758, 277.822), module,
        IsochronicGenerator::FMRATE_CV_IN_INPUT));
    addInput(createInputCentered<PJ301MPort>(
        Vec(257.01, 277.822), module,
        IsochronicGenerator::FMRANGE_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(Vec(185.758, 314.501), module,
                                        IsochronicGenerator::RAMP_CV_IN_INPUT));
    addInput(
        createInputCentered<PJ301MPort>(Vec(185.758, 347.418), module,
                                        IsochronicGenerator::GAP_CV_IN_INPUT));

    addOutput(
        createOutputCentered<PJ301MPort>(Vec(256.014, 313.104), module,
                                         IsochronicGenerator::R_OUT_OUTPUT));
    addOutput(
        createOutputCentered<PJ301MPort>(Vec(256.014, 347.418), module,
                                         IsochronicGenerator::L_OUT_OUTPUT));

    addChild(createLightCentered<MediumLight<RedLight>>(
        Vec(183.061, 92.438), module,
        IsochronicGenerator::BF_LIGHT_LIGHT));
  }

  void appendContextMenu(Menu *menu) override {
    ModuleWidget::appendContextMenu(menu);
    IsochronicGenerator *module =
        dynamic_cast<IsochronicGenerator *>(this->module);
    if (!module) {
      return;
    }

    menu->addChild(new MenuSeparator());
    menu->addChild(createMenuLabel("IsochronicGenerator"));
    menu->addChild(createSubmenuItem(
        "Dial maxima", "",
        [module](Menu *submenu) {
          submenu->addChild(createIndexSubmenuItem(
              "Beat frequency", {"100 Hz", "50 Hz", "20 Hz", "10 Hz"},
              [module]() {
                return static_cast<size_t>(module->persistentState.beatRangeIndex);
              },
              [module](size_t index) {
                module->persistentState.beatRangeIndex =
                    static_cast<int>(index);
                module->applyPersistentState();
              }));
          submenu->addChild(createIndexSubmenuItem(
              "Carrier frequency",
              {"20 kHz", "10 kHz", "5 kHz", "2 kHz", "1 kHz"},
              [module]() {
                return static_cast<size_t>(
                    module->persistentState.carrierRangeIndex);
              },
              [module](size_t index) {
                module->persistentState.carrierRangeIndex =
                    static_cast<int>(index);
                module->applyPersistentState();
              }));
          submenu->addChild(createIndexSubmenuItem(
              "AM rate", {"100 Hz", "50 Hz", "20 Hz", "10 Hz"},
              [module]() {
                return static_cast<size_t>(
                    module->persistentState.amRateRangeIndex);
              },
              [module](size_t index) {
                module->persistentState.amRateRangeIndex =
                    static_cast<int>(index);
                module->applyPersistentState();
              }));
          submenu->addChild(createIndexSubmenuItem(
              "FM rate",
              {"100 Hz", "50 Hz", "20 Hz", "10 Hz", "5 Hz", "2 Hz", "1 Hz"},
              [module]() {
                return static_cast<size_t>(
                    module->persistentState.fmRateRangeIndex);
              },
              [module](size_t index) {
                module->persistentState.fmRateRangeIndex =
                    static_cast<int>(index);
                module->applyPersistentState();
              }));
          submenu->addChild(createIndexSubmenuItem(
              "FM range", {"100 Hz", "50 Hz", "20 Hz", "10 Hz"},
              [module]() {
                return static_cast<size_t>(
                    module->persistentState.fmRangeRangeIndex);
              },
              [module](size_t index) {
                module->persistentState.fmRangeRangeIndex =
                    static_cast<int>(index);
                module->applyPersistentState();
              }));
        }));
    menu->addChild(createIndexSubmenuItem(
        "V/Oct input range", {"-5 V to +5 V", "0 V to +10 V"},
        [module]() {
          return static_cast<size_t>(module->persistentState.vOctRangeIndex);
        },
        [module](size_t index) {
          module->persistentState.vOctRangeIndex = static_cast<int>(index);
          module->applyPersistentState();
        }));
    menu->addChild(createIndexSubmenuItem(
        "Polyphony", {"1 channel (mono)", "2 channels", "3 channels",
                      "4 channels"},
        [module]() {
          return static_cast<size_t>(module->persistentState.polyphonyModeIndex);
        },
        [module](size_t index) {
          module->persistentState.polyphonyModeIndex = static_cast<int>(index);
          module->applyPersistentState();
        }));
  }
};

Model *modelIsochronicGenerator =
    createModel<IsochronicGenerator, IsochronicGeneratorWidget>(
        "IsochronicGenerator");
