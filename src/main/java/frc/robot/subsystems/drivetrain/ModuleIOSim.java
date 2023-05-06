package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.controllers.DieterController;

import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class ModuleIOSim implements ModuleIO {

    private final FlywheelSim driveMotor;
    private final FlywheelSim angleMotor;

    private double setpointAngleRads = 0;
    private double currentAngleRads = 0;
    private final DieterController angleFeedback;
    private double appliedAngleVoltage = 0;
    private double appliedDriveVoltage = 0;

    public ModuleIOSim() {
        driveMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1 / DRIVE_REDUCTION, DRIVE_MOMENT_OF_INERTIA);
        angleMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1 / ANGLE_REDUCTION, ANGLE_MOMENT_OF_INERTIA);

        angleFeedback = new DieterController(1, 0, 0, 0);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        driveMotor.update(0.02);
        angleMotor.update(0.02);

        inputs.angleRads = inputs.angleRads +
                angleMotor.getAngularVelocityRadPerSec() * 0.02;
        inputs.angleRads = MathUtil.angleModulus(inputs.angleRads);
        currentAngleRads = inputs.angleRads;
        inputs.setpointAngleRads = setpointAngleRads;
        inputs.encoderAngleRads = inputs.angleRads;
        inputs.encoderConnected = true;
        inputs.appliedAngleVoltage = appliedAngleVoltage;
        inputs.appliedAngleCurrent = angleMotor.getCurrentDrawAmps();

        inputs.velocityMetersPerSecond = driveMotor.getAngularVelocityRadPerSec()
                * WHEEL_DIAMETER;
        inputs.moduleDistanceMeters = inputs.moduleDistanceMeters +
                inputs.velocityMetersPerSecond * 0.02;
        inputs.appliedDriveVoltage = appliedDriveVoltage;
        inputs.appliedDriveCurrent = driveMotor.getCurrentDrawAmps();
    }

    @Override
    public void setAngle(double angleRads) {
        setpointAngleRads = angleRads;
        appliedAngleVoltage = angleFeedback.calculate(currentAngleRads, angleRads);
        angleMotor.setInputVoltage(appliedAngleVoltage);
    }

    @Override
    public void setDriveVoltage(double voltage) {
        appliedDriveVoltage = voltage;
        driveMotor.setInputVoltage(voltage);
    }

    @Override
    public void configMotionMagic(double[] motionMagicConfigs) {
    }

    @Override
    public boolean encoderJustConnected() {
        return false;
    }

    @Override
    public boolean initializedAngleFalcon() {
        return true;
    }

    @Override
    public void resetAngle() {
    }
}
