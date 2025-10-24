package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
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

    private IntakePivotGoal currentGoal;

    public enum IntakePivotGoal {
        IDLE(() -> 0), // TODO: set idle angle
        TO_INTAKE(() -> 0), // TODO: Set intake angle
        TO_SCOREL1(() -> 0), // TODO: set scoring angle
        TO_INTAKE_HANDOFF(() -> 0); // TODO: set handoff angle

        @Getter
        private DoubleSupplier pivotAngle;

        private IntakePivotGoal(DoubleSupplier pivotAngle) {
            this.pivotAngle = pivotAngle;
        }
    }

    public IntakePivot(IntakePivotIO io) {
        this.io = io;
    }

    public void setGoal(IntakePivotGoal goal){
        currentGoal = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Pivot", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);
    }

    /** Returns true if this subsystem is within a margin of error of the current goal */
    @AutoLogOutput(key = "Intake Pivot/At Goal")
    public boolean atGoal() {
        return false;
    }
}
