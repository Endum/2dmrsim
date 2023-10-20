package io.github.ericmedvet.mrsim2d.core.agents.selfassemblyvsr;

import io.github.ericmedvet.mrsim2d.core.ActionPerformer;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.actions.CreateVoxel;
import io.github.ericmedvet.mrsim2d.core.bodies.Anchorable;
import io.github.ericmedvet.mrsim2d.core.bodies.Body;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import io.github.ericmedvet.mrsim2d.core.engine.ActionException;

import java.util.List;
import java.util.stream.Collectors;

public abstract class AbstractSelfAssemblyVSR implements EmbodiedAgent {

    private final int unitNumber;
    private final Voxel.Material material;
    private final List<Anchorable> unitBody;
    private final double voxelSideLength;
    private final double voxelMass;

    public AbstractSelfAssemblyVSR(
            int  unitNumber,
            Voxel.Material material,
            double voxelSideLength,
            double voxelMass
    ) {
        this.unitNumber = unitNumber;
        this.material = material;
        this.unitBody = List.of();
        this.voxelSideLength = voxelSideLength;
        this.voxelMass = voxelMass;
    }

    @Override
    public void assemble(ActionPerformer actionPerformer) throws ActionException {
        for (int i = 0; i < this.unitNumber; i++) {
            Anchorable body = actionPerformer
                    .perform(new CreateVoxel(
                            this.voxelSideLength,
                            this.voxelMass,
                            this.material))
                    .outcome()
                    .orElseThrow();

            //TODO: Missing initial position of voxels.

            this.unitBody.add(body);
        }
    }

    @Override
    public List<Body> bodyParts() {
        return this.unitBody.stream().map(a -> (Body)a).toList();
    }

}
