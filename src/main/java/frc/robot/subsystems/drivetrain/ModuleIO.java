package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    default void updateInputs(ModuleInputs inputs) {
    }

    default void setAngle(double angleRads) {
    }

    default void setDriveVoltage(double voltage) {
    }

    default void configMotionMagic(double[] motionMagicConfigs) {
    }

    default boolean encoderJustConnected() {
        return false;
    }

    default boolean initializedAngleFalcon() {
        return false;
    }

    default void resetAngle() {
    }

    @AutoLog
    class ModuleInputs {
        public double angleRads = 0;
        public double setpointAngleRads = 0;
        public double encoderAngleRads = 0;
        public boolean encoderConnected = false;
        public double appliedAngleVoltage = 0;
        public double appliedAngleCurrent = 0;

        public double velocityMetersPerSecond = 0;
        public double setpointVelocityMetersPerSecond = 0;
        public double setpointDriveVoltage = 0;
        public double moduleDistanceMeters = 0;
        public double appliedDriveVoltage = 0;
        public double appliedDriveCurrent = 0;

        public double totalCurrentDrawCoulombs = 0;
    }
}
