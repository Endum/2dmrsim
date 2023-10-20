package io.github.ericmedvet.mrsim2d.core.agents.selfassemblyvsr;

import io.github.ericmedvet.jsdynsym.core.DoubleRange;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.mrsim2d.core.Action;
import io.github.ericmedvet.mrsim2d.core.ActionOutcome;
import io.github.ericmedvet.mrsim2d.core.NumMultiBrained;
import io.github.ericmedvet.mrsim2d.core.Sensor;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;

import java.util.List;
import java.util.stream.IntStream;

public class NumSelfAssemblyVSR extends AbstractSelfAssemblyVSR implements NumMultiBrained {

    private static final DoubleRange INPUT_RANGE = DoubleRange.SYMMETRIC_UNIT;
    private static final DoubleRange OUTPUT_RANGE = DoubleRange.SYMMETRIC_UNIT;
    private static final double ATTACH_ACTION_THRESHOLD = 0.25d;
    private final int nSignals;
    private final boolean directionalCommunication;
    private final boolean directionalAttach;
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
            boolean directionalCommunication,
            boolean directionalAttach,
            List<NumericalDynamicalSystem<?>> unitNDS,
            List<List<Sensor<? super Voxel>>> sensors
    ) {
        super(unitNumber, material, voxelSideLength, voxelMass);
        this.nSignals = nSignals;
        this.directionalCommunication = directionalCommunication;
        this.directionalAttach = directionalAttach;
        this.unitNDS = unitNDS;
        this.sensors = sensors;
        this.inputs = new double
                [this.unitNDS.size()]
                [nOfInputs(sensors.size(), nSignals)];
        this.outputs = new double
                [this.unitNDS.size()]
                [nOfOutputs(nSignals, directionalCommunication, directionalAttach)];

        this.unitNDS.forEach(nds -> nds.checkDimension(
                nOfInputs(this.sensors.get(0).size(), nSignals),
                nOfOutputs(nSignals, directionalCommunication, directionalAttach)
        ));
    }

    public static int nOfInputs(int nSensors, int nSignals) {
        return nSensors + nSignals * 4;
    }

    public static int nOfOutputs(int nSignals, boolean dirCom, boolean dirAtt) {
        return
                (dirCom ? 4 * nSignals : nSignals) // Output communications to near units.
                + (dirAtt ? 4 : 1)                 // Attach actuation.
                + 1;                               // Voxel actuation.
    }

    @Override
    public List<? extends Action<?>> act(double t, List<ActionOutcome<?, ?>> previousActionOutcomes) {
        return null;
    }

    @Override
    public List<BrainIO> brainIOs() {
        return null;
        /*IntStream.range(0, this.unitNDS.size()).map(i -> new BrainIO(

        ));*/
    }

    @Override
    public List<NumericalDynamicalSystem<?>> brains() {
        return this.unitNDS;
    }
}
