package frc.robot.subsystems.outtake.endeffector;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
//no hybrid cycles ;(
//beam break on the side
    private EndEffectorIO io;
    private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("End Effector Roller/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("End Effector Roller/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("End Effector Roller/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("End Effector Roller/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("End Effector Roller/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("End Effector Roller/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("End Effector Roller/Gains/kG", kGains.kG());

    public enum EndEffectorRollerGoal {
        IDLE(() -> 0); // Should be the current angle

        @Getter
        private DoubleSupplier rollerVel;

        private EndEffectorRollerGoal(DoubleSupplier rollerVel) {
            this.rollerVel = rollerVel;
        }
    }

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("End Effector Roller", inputs);
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
    @AutoLogOutput(key = "End Effector Roller/At Goal")
    public boolean atGoal() {
        return false;
    }
}
