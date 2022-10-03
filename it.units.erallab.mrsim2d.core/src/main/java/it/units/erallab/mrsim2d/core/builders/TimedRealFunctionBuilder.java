package it.units.erallab.mrsim2d.core.builders;

import it.units.erallab.mrsim2d.builder.Param;
import it.units.erallab.mrsim2d.core.functions.MultiLayerPerceptron;
import it.units.erallab.mrsim2d.core.functions.Sinusoidal;
import it.units.erallab.mrsim2d.core.functions.SteppedTimedRealFunction;
import it.units.erallab.mrsim2d.core.functions.TimedRealFunction;

import java.util.EnumSet;
import java.util.function.BiFunction;

/**
 * @author "Eric Medvet" on 2022/10/03 for 2dmrsim
 */
public class TimedRealFunctionBuilder {
  private TimedRealFunctionBuilder() {
  }

  public static BiFunction<Integer, Integer, MultiLayerPerceptron> mlp(
      @Param(value = "innerLayerRatio", dD = 0.65) double innerLayerRatio,
      @Param(value = "nOfInnerLayers", dI = 1) int nOfInnerLayers,
      @Param(value = "activationFunction", dS = "tanh") String activationFunction
  ) {
    return (nOfInputs, nOfOutputs) -> {
      int[] innerNeurons = new int[nOfInnerLayers];
      int centerSize = (int) Math.max(2, Math.round(nOfInputs * innerLayerRatio));
      if (nOfInnerLayers > 1) {
        for (int i = 0; i < nOfInnerLayers / 2; i++) {
          innerNeurons[i] = nOfInputs + (centerSize - nOfInputs) / (nOfInnerLayers / 2 + 1) * (i + 1);
        }
        for (int i = nOfInnerLayers / 2; i < nOfInnerLayers; i++) {
          innerNeurons[i] =
              centerSize + (nOfOutputs - centerSize) / (nOfInnerLayers / 2 + 1) * (i - nOfInnerLayers / 2);
        }
      } else if (nOfInnerLayers > 0) {
        innerNeurons[0] = centerSize;
      }
      int nOfWeights = MultiLayerPerceptron.countWeights(nOfInputs, innerNeurons, nOfOutputs);
      return new MultiLayerPerceptron(
          MultiLayerPerceptron.ActivationFunction.valueOf(activationFunction.toUpperCase()),
          nOfInputs,
          innerNeurons,
          nOfOutputs
      );
    };
  }

  public static BiFunction<Integer, Integer, Sinusoidal> sinP(
      @Param(value = "f", dD = 1f) double frequency,
      @Param(value = "a", dD = 1f) double amplitude
  ) {
    return (nOfInputs, nOfOutputs) -> {
      Sinusoidal sinusoidal = new Sinusoidal(nOfInputs, nOfOutputs, EnumSet.of(Sinusoidal.Type.PHASE));
      sinusoidal.setFrequencies(frequency);
      sinusoidal.setAmplitudes(amplitude);
      return sinusoidal;
    };
  }

  public static BiFunction<Integer, Integer, Sinusoidal> sinPF(
      @Param(value = "a", dD = 1f) double amplitude
  ) {
    return (nOfInputs, nOfOutputs) -> {
      Sinusoidal sinusoidal = new Sinusoidal(nOfInputs, nOfOutputs, EnumSet.of(Sinusoidal.Type.PHASE));
      sinusoidal.setAmplitudes(amplitude);
      return sinusoidal;
    };
  }

  public static BiFunction<Integer, Integer, Sinusoidal> sinPFA() {
    return (nOfInputs, nOfOutputs) -> new Sinusoidal(
        nOfInputs,
        nOfOutputs,
        EnumSet.of(Sinusoidal.Type.PHASE, Sinusoidal.Type.FREQUENCY, Sinusoidal.Type.AMPLITUDE)
    );
  }

  public static BiFunction<Integer, Integer, SteppedTimedRealFunction> stepped(
      @Param("step") double step,
      @Param("innerFunction") BiFunction<Integer, Integer, ? extends TimedRealFunction> innerFunction
  ) {
    return (nOfInputs, nOfOutputs) -> new SteppedTimedRealFunction(innerFunction.apply(nOfInputs, nOfOutputs), step);
  }
}
