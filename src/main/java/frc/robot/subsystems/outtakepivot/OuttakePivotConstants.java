package frc.robot.subsystems.outtakepivot;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class OuttakePivotConstants {

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                    // TODO: tune
                default -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            };

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                    // TODO: tune
                default -> 0.0;
            };
}
