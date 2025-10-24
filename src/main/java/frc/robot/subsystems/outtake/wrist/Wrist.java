package frc.robot.subsystems.outtake.wrist;

import static frc.robot.subsystems.outtake.wrist.WristConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private double desiredLevel;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/Gains/kG", kGains.kG());
    private static final LoggedTunableNumber L1Tunable = new LoggedTunableNumber("Wrist/Position/L1", 1);
    private static final LoggedTunableNumber LowGoalTunable = new LoggedTunableNumber("Wrist/Position/LowGoal", 2);
    private static final LoggedTunableNumber L4Tunable = new LoggedTunableNumber("Wrist/Position/L4", 3);
    private WristGoal currentGoal = WristGoal.IDLE;

    public enum WristGoal {
        IDLE(() -> 0),
        L1(L1Tunable),
        LowGoal(LowGoalTunable),
        L4(L4Tunable);

        private DoubleSupplier wristAngle;

        private WristGoal(DoubleSupplier wristAngle) {
            this.wristAngle = wristAngle;
        }

        public double getWristAngle() {
            return wristAngle.getAsDouble();
        }
    }

    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs); // Logs the inputs/telemetry data
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD); // Sets PID if changed
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA); // Sets Feedforward if changed
    }

    // Put methods for controlling this subsystem using io interface methods
    public void resetPosition(double angle) {
        io.resetPosition(angle);
    }

    public void stop() {
        io.stop();
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void runPosition(double goal) {
        io.runPosition(goal);
    }

    public void setCurrentGoal(WristGoal goal) {
        currentGoal = goal;
    }

    /**
     * Returns true if this subsystem is within a margin of error of the current
     * goal
     */
    @AutoLogOutput
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(inputs.position, currentGoal.wristAngle.getAsDouble(), kAngleTolerance);
    }
}
