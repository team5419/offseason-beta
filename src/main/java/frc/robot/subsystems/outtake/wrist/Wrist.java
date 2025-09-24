package frc.robot.subsystems.outtake.wrist;

import static frc.robot.subsystems.outtake.wrist.WristConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber l1 = new LoggedTunableNumber("Wrist/Goals/l1", kWristAngles.l1());
    private static final LoggedTunableNumber lowGoal =
            new LoggedTunableNumber("Wrist/Goals/lowGoal", kWristAngles.lowGoal());
    private static final LoggedTunableNumber l4Goal = new LoggedTunableNumber("Wrist/Goals/l4Goal", kWristAngles.l4());
    private static final LoggedTunableNumber stow = new LoggedTunableNumber("Wrist/Goals/stow", kWristAngles.stow());

    public enum WristGoal {
        IDLE(() -> 0),
        L1(l1),
        LOW_GOAL(lowGoal),
        L4(l4Goal),
        STOW(stow);

        private DoubleSupplier wristAngle;

        private WristGoal(java.util.function.DoubleSupplier wristAngle) {
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
        Logger.processInputs("Wrist", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);
    }

    public void resetPosition() {}

    public void stop() {}

    public void runVolts() {}

    public void runPosition() {}

    public void setDesiredLevel(ElevatorGoal goal) {}

    @AutoLogOutput
    public boolean atGoal() {
        return false;
    }
}
