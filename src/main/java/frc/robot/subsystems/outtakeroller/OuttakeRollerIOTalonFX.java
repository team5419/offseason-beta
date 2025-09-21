package frc.robot.subsystems.outtakeroller;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class OuttakeRollerIOTalonFX implements OuttakeRollerIO {

    private TalonFX leaderMotor, followerMotor;

    public OuttakeRollerIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kOuttakeRollerLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kOuttakeRollerFollowerID, GlobalConstants.kCANivoreName);
    }
}
