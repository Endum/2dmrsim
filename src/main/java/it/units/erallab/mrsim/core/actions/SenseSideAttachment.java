/*
 * Copyright 2022 Eric Medvet <eric.medvet@gmail.com> (as eric)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.units.erallab.mrsim.core.actions;

import it.units.erallab.mrsim.core.ActionPerformer;
import it.units.erallab.mrsim.core.Agent;
import it.units.erallab.mrsim.core.SelfDescribedAction;
import it.units.erallab.mrsim.core.bodies.Anchor;
import it.units.erallab.mrsim.core.bodies.Anchorable;
import it.units.erallab.mrsim.core.bodies.Voxel;
import it.units.erallab.mrsim.core.bodies.Voxel.Side;
import it.units.erallab.mrsim.engine.ActionException;
import it.units.erallab.mrsim.util.DoubleRange;

import java.util.Collection;
import java.util.List;

public record SenseSideAttachment(Side side, Voxel body) implements Sense<Voxel>, SelfDescribedAction<Double> {

  @Override
  public Double perform(ActionPerformer performer, Agent agent) throws ActionException {
    //consider side anchors
    Collection<Anchor> anchors = body.anchorsOn(side);
    if (anchors.isEmpty()) {
      return 0d;
    }
    //find "most attached" other body
    List<Anchorable> anchorables = anchors.stream()
        .map(Anchor::attachedAnchorables)
        .flatMap(Collection::stream)
        .toList();
    if (anchorables.isEmpty()) {
      return 0d;
    }
    long maxAttachedAnchorsOfSameBody = anchorables.stream()
        .mapToLong(b -> anchors.stream().filter(a -> a.isAnchoredTo(b)).count())
        .max()
        .orElse(0);
    //return
    return (double) maxAttachedAnchorsOfSameBody / (double) anchors.size();
  }

  @Override
  public DoubleRange range() {
    return DoubleRange.UNIT;
  }
}
