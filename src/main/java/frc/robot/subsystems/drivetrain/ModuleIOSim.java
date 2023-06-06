package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.math.differential.Integral;
import org.eclipse.jetty.util.MathUtils;

import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class ModuleIOSim implements ModuleIO {

    private final FlywheelSim driveMotor;
    private final FlywheelSim angleMotor;

    private double currentAngleRads = 0;
    private double currentDriveVelocity = 0;
    private final PIDController angleFeedback;
    private final PIDController driveVelocityFeedback;

    private double appliedAngleVoltage = 0;
    private double appliedDriveVoltage = 0;

    private final Integral moduleDistance = new Integral(0, 0);
    private final Integral angleRads = new Integral(0, 0);

    public ModuleIOSim() {
        driveMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1 / DRIVE_REDUCTION, DRIVE_MOMENT_OF_INERTIA);
        angleMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1 / ANGLE_REDUCTION, ANGLE_MOMENT_OF_INERTIA);

        angleFeedback = new PIDController(3.5, 0, 0, 0.02);
        driveVelocityFeedback = new PIDController(2.8, 0, 0.03, 0.02);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        driveMotor.update(0.02);
        angleMotor.update(0.02);

        angleRads.update(angleMotor.getAngularVelocityRadPerSec());
        angleRads.override(MathUtil.angleModulus(angleRads.get()));
        inputs.angleRads = angleRads.get();
        currentAngleRads = inputs.angleRads;
        currentDriveVelocity = inputs.velocityMetersPerSecond;
        inputs.encoderAngleRads = inputs.angleRads;
        inputs.encoderConnected = true;
        inputs.appliedAngleVoltage = appliedAngleVoltage;
        inputs.appliedAngleCurrent = angleMotor.getCurrentDrawAmps();

        inputs.velocityMetersPerSecond = driveMotor.getAngularVelocityRadPerSec()
                * WHEEL_DIAMETER / 2;
        moduleDistance.update(inputs.velocityMetersPerSecond);
        inputs.moduleDistanceMeters = moduleDistance.get();
        inputs.appliedDriveVoltage = appliedDriveVoltage;
        inputs.appliedDriveCurrent = driveMotor.getCurrentDrawAmps();
    }

    @Override
    public void setAngle(double angleRads) {
        appliedAngleVoltage = angleFeedback.calculate(currentAngleRads, angleRads);
        angleMotor.setInputVoltage(appliedAngleVoltage);
    }

    @Override
    public void setDriveVelocity(double velocity) {
        appliedDriveVoltage = driveVelocityFeedback.calculate(currentDriveVelocity, velocity) + kV*velocity;
        driveMotor.setInputVoltage(appliedDriveVoltage);
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
