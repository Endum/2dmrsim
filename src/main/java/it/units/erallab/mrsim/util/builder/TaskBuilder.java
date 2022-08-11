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

package it.units.erallab.mrsim.util.builder;

import it.units.erallab.mrsim.core.geometry.Terrain;
import it.units.erallab.mrsim.tasks.locomotion.Locomotion;

import java.util.List;

/**
 * @author "Eric Medvet" on 2022/08/11 for 2dmrsim
 */
public class TaskBuilder extends NamedBuilder<Object> {

  private TaskBuilder() {
    register(List.of("terrain", "t"), TerrainBuilder.getInstance());
    register("locomotion", TaskBuilder::createLocomotion);
  }

  private static Locomotion createLocomotion(ParamMap m, NamedBuilder<?> nb) {
    return new Locomotion(
        m.d("duration", 30),
        (Terrain) nb.build(m.npm("terrain")).orElseThrow(() -> new IllegalArgumentException("No value for terrain")),
        m.d("initialXGap", 1d),
        m.d("initialYGap", 0.25d)
    );
  }

  private final static TaskBuilder INSTANCE = new TaskBuilder();

  public static TaskBuilder getInstance() {
    return INSTANCE;
  }

}
