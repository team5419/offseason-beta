package frc.robot.subsystems.intake.roller;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {

    private IntakeRollerIO io;
    private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake Roller/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake Roller/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake Roller/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake Roller/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake Roller/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake Roller/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake Roller/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber intake = new LoggedTunableNumber("Roller/Intake", -80.0);
    private static final LoggedTunableNumber outtakeEndEffector =
            new LoggedTunableNumber("Roller/OuttakeEndEffector", 20.0);
    private static final LoggedTunableNumber outtakeCorral = new LoggedTunableNumber("Roller/OuttakeRoller", 10.0);

    public enum IntakeRollerGoal {
        IDLE(() -> 0), // Should be the current angle
        INTAKE(intake),
        OUTTAKEENDEFFECTOR(outtakeEndEffector),
        OUTTAKECORRAL(outtakeCorral);

        @Getter
        private DoubleSupplier rollerVel;

        private IntakeRollerGoal(DoubleSupplier rollerVel) {
            this.rollerVel = rollerVel;
        }
    }

    IntakeRollerGoal currentGoal = IntakeRollerGoal.IDLE;

    public IntakeRoller(IntakeRollerIO io) {
        this.io = io;
    }

    public void setGoal(IntakeRollerGoal goal) {
        currentGoal = goal;
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
        io.runVelocity(currentGoal.getRollerVel().getAsDouble());
        if (currentGoal == IntakeRollerGoal.IDLE) {
            io.stop();
        } else {
            io.runVelocity(currentGoal.getRollerVel().getAsDouble());
        }
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
