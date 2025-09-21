package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber stowHeight = new LoggedTunableNumber("Elevator/Stow Height", kStowHeight);
    private static final LoggedTunableNumber l2 = new LoggedTunableNumber("Elevator/L2", kL2Height);
    private static final LoggedTunableNumber l3 = new LoggedTunableNumber("Elevator/L3", kL3Height);
    private static final LoggedTunableNumber l4 = new LoggedTunableNumber("Elevator/L4", kL4Height);

    public enum ElevatorGoal {
        IDLE(() -> 0), // Should be the current height
        STOW(stowHeight),
        L1(stowHeight),
        L2(l2),
        L3(l3),
        L4(l4);

        @Getter
        private DoubleSupplier eleHeight;

        private ElevatorGoal(DoubleSupplier eleHeight) {
            this.eleHeight = eleHeight;
        }
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
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

    public void setDesiredLevel(ElevatorGoal goal) {}

    public boolean atGoal() {
        return false;
    }
}
