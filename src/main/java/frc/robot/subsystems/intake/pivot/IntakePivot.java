package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {

    private IntakePivotIO io;
    private IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake Pivot/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake Pivot/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake Pivot/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake Pivot/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake Pivot/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake Pivot/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake Pivot/Gains/kG", kGains.kG());

    public enum IntakePivotGoal {
        IDLE(() -> 0), // TODO: set idle angle
        INTAKE(() -> 17), // TODO: Set intake angle
        SCORE_L1(() -> 0), // TODO: set scoring angle
        HANDOFF(() -> 0); // TODO: set handoff angle

        @Getter
        private DoubleSupplier pivotAngle;

        private IntakePivotGoal(DoubleSupplier pivotAngle) {
            this.pivotAngle = pivotAngle;
        }
    }

    @Getter
    @Setter
    private IntakePivotGoal currentGoal = IntakePivotGoal.IDLE;

    public IntakePivot(IntakePivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Pivot", inputs);
        Logger.recordOutput("Intake Pivot/Goal", currentGoal);

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);

        if (currentGoal == IntakePivotGoal.IDLE) {
            stop();
            return;
        } else {
            io.runPosition(currentGoal.getPivotAngle().getAsDouble());
        }
    }

    @AutoLogOutput(key = "Intake Pivot/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                inputs.position, currentGoal.getPivotAngle().getAsDouble());
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void stop() {
        io.stop();
    }
}
