/*
 * Copyright 2022 eric
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

package it.units.erallab.mrsim2d.buildable.builders;

import io.github.ericmedvet.jnb.core.Param;
import it.units.erallab.mrsim2d.core.Sensor;
import it.units.erallab.mrsim2d.core.agents.gridvsr.GridBody;
import it.units.erallab.mrsim2d.core.bodies.Voxel;
import it.units.erallab.mrsim2d.core.util.Grid;

import java.util.List;
import java.util.function.Function;

public class VSRMisc {
  private VSRMisc() {
  }

  @SuppressWarnings("unused")
  public static GridBody gridBody(
      @Param("shape") Grid<Boolean> shape,
      @Param("sensorizingFunction") Function<Grid<Boolean>, Grid<List<Sensor<? super Voxel>>>> sensorizingFunction
  ) {
    return new GridBody(shape, sensorizingFunction);
  }
}
