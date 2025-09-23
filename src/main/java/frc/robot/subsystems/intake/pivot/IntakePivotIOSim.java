package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {

    private final SingleJointedArmSim pivotSim;
    private final PIDController controller;

    public IntakePivotIOSim() {
        // Initialize variables with default/mock values
        pivotSim = new SingleJointedArmSim(
                null,    // placeholder for motor
                1.0,     // gear ratio placeholder
                1.0,     // moment of inertia placeholder
                0.0,     // length placeholder
                0.0,     // min angle placeholder
                180.0,   // max angle placeholder
                false,   // simulation flag
                180.0    // initial angle placeholder
        );

        controller = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        // TODO: implement
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // TODO: implement
    }

    @Override
    public void runPosition(double degrees) {
        // TODO: implement
    }

    @Override
    public void runVolts(double volts) {
        // TODO: implement
    }

    @Override
    public void setPID(double P, double I, double D) {
        // TODO: implement
    }

    @Override
    public void stop() {
        // TODO: implement
    }

    @Override
    public void resetPosition(double degrees) {
        // TODO: implement
    }
}
