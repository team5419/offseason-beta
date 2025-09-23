package frc.robot.subsystems.outtakepivot;

import static frc.robot.subsystems.outtakepivot.OuttakePivotConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class OuttakePivot extends SubsystemBase {

    private OuttakePivotIO io;
    private OuttakePivotIOInputsAutoLogged inputs = new OuttakePivotIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Outtake Pivot/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Outtake Pivot/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Outtake Pivot/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Outtake Pivot/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Outtake Pivot/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Outtake Pivot/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Outtake Pivot/Gains/kG", kGains.kG());

    public OuttakePivot(OuttakePivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Outtake Pivot", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);
    }
}
