/*-
 * ========================LICENSE_START=================================
 * mrsim2d-core
 * %%
 * Copyright (C) 2020 - 2023 Eric Medvet
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package io.github.ericmedvet.mrsim2d.core.agents.selfassemblyvsr;

import io.github.ericmedvet.jsdynsym.core.DoubleRange;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.mrsim2d.core.Action;
import io.github.ericmedvet.mrsim2d.core.ActionOutcome;
import io.github.ericmedvet.mrsim2d.core.NumMultiBrained;
import io.github.ericmedvet.mrsim2d.core.Sensor;
import io.github.ericmedvet.mrsim2d.core.actions.*;
import io.github.ericmedvet.mrsim2d.core.bodies.Anchor;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import io.github.ericmedvet.mrsim2d.core.geometry.Point;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.IntStream;

public class NumSelfAssemblyVSR extends AbstractSelfAssemblyVSR implements NumMultiBrained {

  private static final DoubleRange INPUT_RANGE = DoubleRange.SYMMETRIC_UNIT;
  private static final DoubleRange OUTPUT_RANGE = DoubleRange.SYMMETRIC_UNIT;
  private static final double ATTACH_ACTION_THRESHOLD = 0.25d;
  private final int nSignals;
  // private final boolean directionalCommunication;
  // private final boolean directionalAttach;
  private final List<NumericalDynamicalSystem<?>> unitNDS;
  private final List<List<Sensor<? super Voxel>>> sensors;
  private final double[][] inputs;
  private final double[][] outputs;

  public NumSelfAssemblyVSR(
      int unitNumber,
      Voxel.Material material,
      double voxelSideLength,
      double voxelMass,
      int nSignals,
      // boolean directionalCommunication,
      // boolean directionalAttach,
      List<NumericalDynamicalSystem<?>> unitNDS,
      List<List<Sensor<? super Voxel>>> sensors) {
    super(unitNumber, material, voxelSideLength, voxelMass);
    this.nSignals = nSignals;
    // this.directionalCommunication = directionalCommunication;
    // this.directionalAttach = directionalAttach;
    this.unitNDS = unitNDS;
    this.sensors = sensors;
    this.inputs = new double[unitNumber][nOfInputs(sensors.size(), nSignals)];
    this.outputs =
        new double[unitNumber]
            [nOfOutputs(nSignals /*, directionalCommunication, directionalAttach*/)];

    this.unitNDS.forEach(
        nds ->
            nds.checkDimension(
                nOfInputs(this.sensors.get(0).size(), nSignals),
                nOfOutputs(nSignals /*, directionalCommunication, directionalAttach*/)));
  }

  public static int nOfInputs(int nSensors, int nSignals) {
    return nSensors + nSignals * 4;
  }

  public static int nOfOutputs(int nSignals /*, boolean dirCom, boolean dirAtt*/) {
    return (
        /*dirCom ?*/ 4 * nSignals /* : nSignals*/) // Output communications to near units.
        + (
        /*dirAtt ? */ 4 /*: 1*/) // Attach actuation.
        + 1; // Voxel actuation.
  }

  @Override
  public List<? extends Action<?>> act(double t, List<ActionOutcome<?, ?>> previousActionOutcomes) {
    // Read inputs.
    int PAOi = 0;
    for (var ndsIn : inputs) {
      for (int i = 0; i < ndsIn.length; i++) {
        ActionOutcome<?, ?> outcome = previousActionOutcomes.get(PAOi);
        if (outcome.action() instanceof Sense<?>) {
          @SuppressWarnings("unchecked")
          ActionOutcome<? extends Sense<Voxel>, Double> o =
              (ActionOutcome<? extends Sense<Voxel>, Double>) outcome;
          ndsIn[i] = INPUT_RANGE.denormalize(o.action().range().normalize(o.outcome().orElse(0d)));
          PAOi++;
        }
      }
    }

    // Compute actuation.
    IntStream.range(0, outputs.length)
        .forEach(
            i ->
                outputs[i] =
                    Arrays.stream(unitNDS.get(i).step(t, inputs[i]))
                        .map(OUTPUT_RANGE::clip)
                        .toArray());

    // Generate next sensors sense actions.
    List<Action<?>> actions =
        new ArrayList<>(
            IntStream.range(0, unitNumber)
                .mapToObj(i -> sensors.get(i).stream().map(s -> s.apply(unitBody.get(i))).toList())
                .flatMap(Collection::stream)
                .toList());

    final AtomicInteger aI = new AtomicInteger(0); // Action index.
    // Generate actuation actions.
    actions.addAll(
        IntStream.range(0, unitNumber)
            .mapToObj(i -> new ActuateVoxel(unitBody.get(i), outputs[i][aI.get()]))
            .toList());
    // Generate attach actions.
    for (var side : Voxel.Side.values()) {
      aI.incrementAndGet();
      for (int i = 0; i < unitNumber; i++) {
        double m = outputs[i][aI.get()];
        if (m > ATTACH_ACTION_THRESHOLD)
          actions.add(
              new AttractAndLinkClosestAnchorable(
                  unitBody.get(i).anchorsOn(side), 1, Anchor.Link.Type.SOFT));
        else if (m < -ATTACH_ACTION_THRESHOLD)
          actions.add(new DetachAnchors(unitBody.get(i).anchorsOn(side)));
      }
    }
    // Generate communication sense and emit actions.
    aI.incrementAndGet();
    for (var side : Voxel.Side.values()) {
      for (int i = 0; i < unitNumber; i++) {
        Point center = unitBody.get(i).poly().center();
        Point mid = unitBody.get(i).side(side).center().diff(center);
        double dir = mid.direction();
        for (int j = 0; j < nSignals; j++) {
          actions.add(
              new EmitNFCMessage(unitBody.get(i), mid, dir, (short) j, outputs[i][aI.get() + j]));
          actions.add(new SenseNFC(unitBody.get(i), mid, dir, (short) j));
        }
      }
      aI.addAndGet(nSignals);
    }

    return actions;
  }

  @Override
  public List<BrainIO> brainIOs() {
    return IntStream.range(0, this.unitNDS.size())
        .mapToObj(
            i ->
                new BrainIO(
                    new RangedValues(inputs[i], INPUT_RANGE),
                    new RangedValues(outputs[i], OUTPUT_RANGE)))
        .toList();
  }

  @Override
  public List<NumericalDynamicalSystem<?>> brains() {
    return this.unitNDS;
  }
}
