package frc.robot.subsystems.intake.roller;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
//Pattern of beam breaks on roof
    private IntakeRollerIO io;
    private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake Roller/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake Roller/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake Roller/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake Roller/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake Roller/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake Roller/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake Roller/Gains/kG", kGains.kG());

    public enum IntakePivotGoal {
        IDLE(() -> 0); // Should be the current angle

        @Getter
        private DoubleSupplier rollerVel;

        private IntakePivotGoal(DoubleSupplier rollerVel) {
            this.rollerVel = rollerVel;
        }
    }

    public IntakeRoller(IntakeRollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Roller", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);
    }

    /**
     * Returns true if this subsystem is within a margin of error of the current
     * goal
     */
    @AutoLogOutput(key = "Intake Roller/At Goal")
    public boolean atGoal() {
        return false;
    }
}
