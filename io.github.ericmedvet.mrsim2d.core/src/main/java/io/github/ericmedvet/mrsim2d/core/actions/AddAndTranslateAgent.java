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

package io.github.ericmedvet.mrsim2d.core.actions;

import io.github.ericmedvet.mrsim2d.core.ActionPerformer;
import io.github.ericmedvet.mrsim2d.core.Agent;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.SelfDescribedAction;
import io.github.ericmedvet.mrsim2d.core.engine.ActionException;
import io.github.ericmedvet.mrsim2d.core.geometry.Point;

public record AddAndTranslateAgent(EmbodiedAgent agent, Point translation)
    implements SelfDescribedAction<EmbodiedAgent> {
  @Override
  public EmbodiedAgent perform(ActionPerformer performer, Agent agent) throws ActionException {
    EmbodiedAgent embodiedAgent =
        (EmbodiedAgent)
            performer
                .perform(new AddAgent(agent()), agent)
                .outcome()
                .orElseThrow(() -> new ActionException(this, "Undoable addition"));
    performer.perform(new TranslateAgent(embodiedAgent, translation), agent);
    return embodiedAgent;
  }
}
