package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import frc.robot.utils.units.UnitModel;

import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class SwerveModuleUtils {

    private static final UnitModel driveFalconToMeters = new UnitModel(
            TICKS_PER_ROTATION / (Math.PI * WHEEL_DIAMETER) / DRIVE_REDUCTION);
    private static final UnitModel angleFalconToRads = new UnitModel(
            TICKS_PER_ROTATION / Constants.TWO_PI / ANGLE_REDUCTION);

    public static double getModuleDistance(double falconTicks) {
        return driveFalconToMeters.toUnits(falconTicks);
    }

    public static double getModuleVelocity(double falconTicksPer10ms) {
        return driveFalconToMeters.toVelocity(falconTicksPer10ms);
    }

    public static double getModuleAngleFromEncoder(double encoderCycles) {
        return encoderCycles * Constants.TWO_PI;
    }

    public static double getFalconTicksFromAngle(double angleRads) {
        return angleFalconToRads.toTicks(angleRads);
    }

    public static double getFalconTicksFromEncoder(double encoderCycles) {
        return getFalconTicksFromAngle(getModuleAngleFromEncoder(encoderCycles));
    }

    public static double getModuleAngleFromFalcon(double falconTicks) {
        return angleFalconToRads.toUnits(falconTicks);
    }
}
