
package io.github.ericmedvet.mrsim2d.core.tasks.locomotion;

import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.Snapshot;
import io.github.ericmedvet.mrsim2d.core.actions.AddAgent;
import io.github.ericmedvet.mrsim2d.core.actions.CreateUnmovableBody;
import io.github.ericmedvet.mrsim2d.core.actions.TranslateAgent;
import io.github.ericmedvet.mrsim2d.core.bodies.Body;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.geometry.BoundingBox;
import io.github.ericmedvet.mrsim2d.core.geometry.Point;
import io.github.ericmedvet.mrsim2d.core.geometry.Terrain;
import io.github.ericmedvet.mrsim2d.core.tasks.AgentsObservation;
import io.github.ericmedvet.mrsim2d.core.tasks.Outcome;
import io.github.ericmedvet.mrsim2d.core.tasks.Task;
import io.github.ericmedvet.mrsim2d.core.util.PolyUtils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Locomotion implements Task<Supplier<EmbodiedAgent>, Outcome<AgentsObservation>> {

  private final static double INITIAL_X_GAP = 1;
  private final static double INITIAL_Y_GAP = 0.25;
  private final double duration;
  private final Terrain terrain;
  private final double initialXGap;
  private final double initialYGap;
  public Locomotion(double duration, Terrain terrain, double initialXGap, double initialYGap) {
    this.duration = duration;
    this.terrain = terrain;
    this.initialXGap = initialXGap;
    this.initialYGap = initialYGap;
  }

  public Locomotion(
      double duration,
      Terrain terrain
  ) {
    this(duration, terrain, INITIAL_X_GAP, INITIAL_Y_GAP);
  }

  @Override
  public Outcome<AgentsObservation> run(
      Supplier<EmbodiedAgent> embodiedAgentSupplier,
      Engine engine,
      Consumer<Snapshot> snapshotConsumer
  ) {
    //create agent
    EmbodiedAgent embodiedAgent = embodiedAgentSupplier.get();
    //build world
    engine.perform(new CreateUnmovableBody(terrain.poly()));
    engine.perform(new AddAgent(embodiedAgent));
    //place agent
    BoundingBox agentBB = embodiedAgent.boundingBox();
    engine.perform(new TranslateAgent(embodiedAgent, new Point(
        terrain.withinBordersXRange().min() + initialXGap - agentBB.min().x(),
        0
    )));
    agentBB = embodiedAgent.boundingBox();
    double maxY = terrain.maxHeightAt(agentBB.xRange());
    engine.perform(new TranslateAgent(embodiedAgent, new Point(
        0,
        maxY + initialYGap - agentBB.min().y()
    )));
    //run for defined time
    Map<Double, AgentsObservation> observations = new HashMap<>();
    while (engine.t() < duration) {
      Snapshot snapshot = engine.tick();
      snapshotConsumer.accept(snapshot);
      observations.put(
          engine.t(),
          new AgentsObservation(
              List.of(new AgentsObservation.Agent(
                  embodiedAgent.bodyParts().stream().map(Body::poly).toList(),
                  PolyUtils.maxYAtX(terrain.poly(), embodiedAgent.boundingBox().center().x())
              ))
          )
      );
    }
    //return
    return new Outcome<>(new TreeMap<>(observations));
  }
}
