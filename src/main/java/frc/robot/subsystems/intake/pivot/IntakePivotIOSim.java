package frc.robot.subsystems.intake.pivot;

import static frc.robot.constants.GlobalConstants.*;
import static frc.robot.subsystems.intake.pivot.IntakePivotConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private final SingleJointedArmSim pivotSim;

    private PIDController controller;
    private double appliedVolts = 0.0;

    public IntakePivotIOSim() {
        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(2),
                310 / 3,
                SingleJointedArmSim.estimateMOI(0.3, 10),
                .3,
                Units.degreesToRadians(0),
                Units.degreesToRadians(150),
                false,
                Units.degreesToRadians(0));

        controller = new PIDController(kGains.kP(), kGains.kI(), kGains.kD());
        resetPosition(0);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        pivotSim.update(kLooperDT);
        inputs.position = Units.radiansToDegrees(pivotSim.getAngleRads());
        inputs.velocity = Units.radiansToDegrees(pivotSim.getVelocityRadPerSec());
        inputs.supplyCurrentAmps = pivotSim.getCurrentDrawAmps();
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void runPosition(double degrees) {
        appliedVolts = controller.calculate(pivotSim.getAngleRads(), Units.degreesToRadians(degrees));
        pivotSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = volts;
        pivotSim.setInputVoltage(volts);
    }

    @Override
    public void setPID(double P, double I, double D) {
        controller.setPID(P, I, D);
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
        pivotSim.setInputVoltage(0);
    }

    @Override
    public void resetPosition(double degrees) {
        pivotSim.setState(Units.degreesToRadians(degrees), 0);
    }
}
