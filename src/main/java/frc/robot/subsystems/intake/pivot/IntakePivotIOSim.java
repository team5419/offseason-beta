package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {

    private final SingleJointedArmSim pivotSim;
    private final PIDController controller;

    private final DCMotorSim sim;
    private double appliedVoltage = 0.0;

    public IntakePivotIOSim(DCMotor motor, double reduction, double moi) {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
        // Initialize variables with default/mock values
        pivotSim = new SingleJointedArmSim(
                null, // placeholder for motor
                1.0, // gear ratio placeholder
                1.0, // moment of inertia placeholder
                0.0, // length placeholder
                0.0, // min angle placeholder
                180.0, // max angle placeholder
                false, // simulation flag
                180.0 // initial angle placeholder
                );

        controller = new PIDController(0.0, 0.0, 0.0);
    }

    // Updates the record with simulated data values
    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        sim.update(0.02);

        inputs.appliedVolts = appliedVoltage;
    }

    @Override
    public void runVolts(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stop() {
        runVolts(0.0);
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
    public void setPID(double P, double I, double D) {
        // TODO: implement
    }

    @Override
    public void resetPosition(double degrees) {
        // TODO: implement
    }
}
