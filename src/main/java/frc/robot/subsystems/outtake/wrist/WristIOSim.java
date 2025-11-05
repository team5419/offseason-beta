package frc.robot.subsystems.outtake.wrist;

import static frc.robot.constants.GlobalConstants.*;
import static frc.robot.subsystems.outtake.wrist.WristConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
    private final SingleJointedArmSim pivotSim;

    private PIDController controller;

    public WristIOSim() {
        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(2),
                310 / 3,
                SingleJointedArmSim.estimateMOI(0.3, 10),
                .3,
                Units.degreesToRadians(kBottomDegree),
                Units.degreesToRadians(kTopDegree),
                false,
                Units.degreesToRadians(0));

        controller = new PIDController(kGains.kP(), kGains.kI(), kGains.kD());
        resetPosition(0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        pivotSim.update(kLooperDT);
        inputs.position = Units.radiansToDegrees(pivotSim.getAngleRads());
        inputs.velocity = Units.radiansToDegrees(pivotSim.getVelocityRadPerSec());
        inputs.supplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    }

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void runPosition(double degrees) {
        pivotSim.setInputVoltage(controller.calculate(pivotSim.getAngleRads(), Units.degreesToRadians(degrees)));
    }

    @Override
    public void runVolts(double volts) {
        pivotSim.setInputVoltage(volts);
    }

    @Override
    public void setPID(double P, double I, double D) {
        controller.setPID(P, I, D);
    }

    @Override
    public void stop() {
        pivotSim.setInputVoltage(0);
    }

    @Override
    public void resetPosition(double degrees) {
        pivotSim.setState(Units.degreesToRadians(degrees), 0);
    }
}
