package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    void updateInputs(SwerveModuleInputs inputs);

    default double getAngle(){
        return 0;
    }

    void setAngle(double angle);

    default double getVelocity(){
        return 0;
    }

    void setVelocity(double velocity);

    default SwerveModulePosition getModulePosition(){
        return null;
    }

    default void updateOffset(double offset){
    }

    default void neutralOutput(){
    }


    @AutoLog
    class SwerveModuleInputs {
        public double driveMotorVelocity;
        public double driveMotorVelocitySetpoint;
        public double driveMotorSupplyCurrent;
        public double driveMotorStatorCurrent;
        public double driveMotorSupplyCurrentOverTime;
        public double driveMotorStatorCurrentOverTime;
        public double driveMotorPosition;
        public double driveMotorAppliedVoltage;

        public double angle;
        public double angleSetpoint;
        public double absolutePosition;
        public double angleMotorVelocity;
        public double angleMotorSupplyCurrent;
        public double angleMotorStatorCurrent;
        public double angleMotorSupplyCurrentOverTime;
        public double angleMotorStatorCurrentOverTime;
        public double angleMotorPosition;
        public double angleMotorAppliedVoltage;

        public double moduleDistance;
    }
}
