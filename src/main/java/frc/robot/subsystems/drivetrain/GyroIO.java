package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
        public boolean gyroConnected = false;

        public double yawPositionRad = 0.0;

        public double rawRollPositionRad = 0.0;
        public double rawPitchPositionRad = 0.0;
        public double rawYawPositionRad = 0.0;

        public double rollVelocityRadPerSec = 0.0;
        public double pitchVelocityRadPerSec = 0.0;
        public double yawVelocityRadPerSec = 0.0;

        public double gyroVelocityX = 0.0;
        public double gyroVelocityY = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}
}
