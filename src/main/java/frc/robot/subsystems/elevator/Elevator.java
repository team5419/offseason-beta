package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
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

    private static final LoggedTunableNumber stow =
            new LoggedTunableNumber("Elevator/Stow Height", kElevatorHeights.stow());
    private static final LoggedTunableNumber l2 = new LoggedTunableNumber("Elevator/L2", kElevatorHeights.l2());
    private static final LoggedTunableNumber l3 = new LoggedTunableNumber("Elevator/L3", kElevatorHeights.l3());
    private static final LoggedTunableNumber l4 = new LoggedTunableNumber("Elevator/L4", kElevatorHeights.l4());

    @Getter
    @Setter
    @AutoLogOutput(key = "Elevator/Current Goal")
    private ElevatorGoal currentGoal = ElevatorGoal.STOW;

    public enum ElevatorGoal {
        IDLE(() -> 0), // Should be the current height
        STOW(stow),
        L1(stow),
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

    public void setDesiredLevel(ElevatorGoal goal) {}

    @AutoLogOutput(key = "Elevator/At Goal")
    public boolean atGoal() {
        return false;
    }
}
