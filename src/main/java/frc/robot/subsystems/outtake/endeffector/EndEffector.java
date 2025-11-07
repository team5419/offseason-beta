package frc.robot.subsystems.outtake.endeffector;

import static frc.robot.subsystems.outtake.endeffector.EndEffectorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

    private EndEffectorIO io;
    private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("End Effector Roller/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("End Effector Roller/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("End Effector Roller/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("End Effector Roller/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("End Effector Roller/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("End Effector Roller/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("End Effector Roller/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber outtake =
            new LoggedTunableNumber("End Effector Roller/Setpoint/Outtake", 10);
    private static final LoggedTunableNumber handoff =
            new LoggedTunableNumber("End Effector Roller/Setpoint/handoff", 10);
    private static final LoggedTunableNumber intakeAlgae =
            new LoggedTunableNumber("End Effector Roller/Setpoint/Intake Algae", 10);
    private static final LoggedTunableNumber holdAlgae =
            new LoggedTunableNumber("End Effector Roller/Setpoint/Hold Algae", 10);

    public enum EndEffectorRollerGoal {
        IDLE(() -> 0),
        HANDOFF(handoff),
        INTAKE_ALGAE(intakeAlgae),
        HOLD_ALGAE(holdAlgae),
        OUTTAKE(outtake);

        @Getter
        private DoubleSupplier rollerVel;

        private EndEffectorRollerGoal(DoubleSupplier rollerVel) {
            this.rollerVel = rollerVel;
        }
    }

    @Getter
    @Setter
    private EndEffectorRollerGoal currentGoal = EndEffectorRollerGoal.IDLE;

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

        if (currentGoal == EndEffectorRollerGoal.IDLE) {
            stop();
        } else {
            io.runVelocity(currentGoal.getRollerVel().getAsDouble());
        }
    }

    public void run(double volts) {
        io.runVolts(volts);
    }

    public void stop() {
        io.stop();
    }

    /**
     * Returns true if this subsystem is within a margin of error of the current
     * goal
     */
    @AutoLogOutput(key = "End Effector Roller/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                ((inputs.velocity[0] + inputs.velocity[1]) / 2),
                currentGoal.getRollerVel().getAsDouble(),
                kVelocityTolerance);
    }
}
