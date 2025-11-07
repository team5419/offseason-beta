package frc.robot.subsystems.outtake.wrist;

import static frc.robot.subsystems.outtake.wrist.WristConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

    private WristIO io;

    @Getter
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber l1 = new LoggedTunableNumber("Wrist/Setpoints/l1", 0);
    private static final LoggedTunableNumber l2 = new LoggedTunableNumber("Wrist/Setpoints/l2", 3);
    private static final LoggedTunableNumber l3 = new LoggedTunableNumber("Wrist/Setpoints/l3", 2);
    private static final LoggedTunableNumber l4 = new LoggedTunableNumber("Wrist/Setpoints/l4", 1);
    private static final LoggedTunableNumber handoff = new LoggedTunableNumber("Wrist/Setpoints/handoff", -60);
    private static final LoggedTunableNumber algae = new LoggedTunableNumber("Wrist/Setpoints/algae", 80);

    public enum WristGoal {
        IDLE(() -> 0),
        L1(l1),
        L2(l2),
        L3(l3),
        L4(l4),
        HANDOFF(handoff),
        ALGAE(algae);

        private DoubleSupplier wristAngle;

        private WristGoal(DoubleSupplier wristAngle) {
            this.wristAngle = wristAngle;
        }

        public double getWristAngle() {
            return wristAngle.getAsDouble();
        }
    }

    @Getter
    @Setter
    private WristGoal currentGoal = WristGoal.IDLE;

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

        if (currentGoal == WristGoal.IDLE) {
            io.stop();
        } else {
            io.runPosition(currentGoal.getWristAngle());
        }
    }

    // Put methods for controlling this subsystem using io interface methods
    public void resetPosition() {}

    public void stop() {}

    public void runVolts() {}

    public void setDesiredLevel(ElevatorGoal goal) {}

    @AutoLogOutput
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(currentGoal.getWristAngle(), inputs.position, kAngleTolerance);
    }

    public static WristGoal goal(ElevatorGoal eleGoal) {
        return switch (eleGoal) {
            case IDLE -> WristGoal.IDLE;
            case STOW -> WristGoal.HANDOFF;
            case L1 -> WristGoal.L1;
            case L2 -> WristGoal.L2;
            case L3 -> WristGoal.L3;
            case L4 -> WristGoal.L4;
        };
    }
}
