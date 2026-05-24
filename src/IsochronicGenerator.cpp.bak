#include "plugin.hpp"

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

  IsochronicGenerator() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    configParam(V_OCT_IN_INPUT, 0.f, 1.f, 0.f, ""); // V Oct In
    configParam(ALTERNATE_MODE_SWITCH_PARAM, 0.f, 1.f, 0.f, "");
    configParam(BF_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(BF_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(CF_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(AM_RATE_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(AM_DEPTH_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(FM_RATE_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(FM_RANGE_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(AM_RATE_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(AM_DEPTH_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(FM_RATE_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(FM_RANGE_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(GAP_P_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(RAMP_P_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(RAMP_P_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configParam(GAP_P_CV_KNOB_PARAM, 0.f, 1.f, 0.f, "");
    configInput(BF_CV_IN_INPUT, "");
    configInput(AMR_CV_IN_INPUT, "");
    configInput(AMD_CV_IN_INPUT, "");
    configInput(FMRATE_CV_IN_INPUT, "");
    configInput(FMRANGE_CV_IN_INPUT, "");
    configInput(RAMP_CV_IN_INPUT, "");
    configInput(GAP_CV_IN_INPUT, "");
    configOutput(R_OUT_OUTPUT, "");
    configOutput(L_OUT_OUTPUT, "");
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
