package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.math.differential.Integral;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Ports.SwerveDrive.*;
import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class Module extends SubsystemBase {

    private final int number;
    private final ModuleIO io;
    private final ModuleInputsAutoLogged inputs;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;

    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private final TunableNumber kF;
    private final TunableNumber sCurveStrength;
    private final TunableNumber cruiseVelocity;
    private final TunableNumber maxAcceleration;
    private final TunableNumber allowableError;
    private final TunableNumber maxIntegralAccumulator;
    private final TunableNumber peakOutput;

    private final Integral currentDrawCoulombs = new Integral(0, 0);

    public Module(ModuleIO io, int number, double[] motionMagicConfig) {
        this.number = number;
        this.io = io;
        io.configMotionMagic(motionMagicConfig);
        inputs = new ModuleInputsAutoLogged();

        if (Robot.isReal()) {
            driveFeedforward = new SimpleMotorFeedforward(
                    STATIC_FEEDFORWARD_REAL,
                    VELOCITY_FEEDFORWARD_REAL,
                    ACCELERATION_FEEDFORWARD_REAL
            );
        } else {
            driveFeedforward = new SimpleMotorFeedforward(
                    STATIC_FEEDFORWARD_SIM,
                    VELOCITY_FEEDFORWARD_SIM,
                    ACCELERATION_FEEDFORWARD_SIM
            );
        }

        driveFeedback = new PIDController(
                DRIVE_Kp,
                DRIVE_Ki,
                DRIVE_Kd,
                0.02
        );

        kP = new TunableNumber(moduleNameOf(number) + "/kP", motionMagicConfig[0]);
        kI = new TunableNumber(moduleNameOf(number) + "/kI", motionMagicConfig[1]);
        kD = new TunableNumber(moduleNameOf(number) + "/kD", motionMagicConfig[2]);
        kF = new TunableNumber(moduleNameOf(number) + "/kF", motionMagicConfig[3]);
        sCurveStrength = new TunableNumber(moduleNameOf(number) + "/sCurveStrength", motionMagicConfig[4]);
        cruiseVelocity = new TunableNumber(moduleNameOf(number) + "/cruiseVelocity", motionMagicConfig[5]);
        maxAcceleration = new TunableNumber(moduleNameOf(number) + "/maxAcceleration", motionMagicConfig[6]);
        allowableError = new TunableNumber(moduleNameOf(number) + "/allowableError", motionMagicConfig[7]);
        maxIntegralAccumulator = new TunableNumber(moduleNameOf(number) + "/maxIntegralAccumulator", motionMagicConfig[8]);
        peakOutput = new TunableNumber(moduleNameOf(number) + "/peakOutput", motionMagicConfig[9]);
    }

    public void set(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(inputs.angleRads));

        double driveVoltage =
                driveFeedforward.calculate(desiredState.speedMetersPerSecond,
                driveFeedback.calculate(inputs.velocityMetersPerSecond, desiredState.speedMetersPerSecond));

        inputs.setpointVelocityMetersPerSecond = desiredState.speedMetersPerSecond
                * Math.cos(desiredState.angle.getRadians() - inputs.angleRads);
        inputs.setpointDriveVoltage = driveVoltage;
        inputs.setpointAngleRads = MathUtil.angleModulus(desiredState.angle.getRadians());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                inputs.velocityMetersPerSecond,
                new Rotation2d(inputs.angleRads));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                inputs.moduleDistanceMeters,
                new Rotation2d(inputs.angleRads));
    }

    public void stop() {
        inputs.setpointDriveVoltage = 0;
        inputs.setpointVelocityMetersPerSecond = 0;
        inputs.setpointAngleRads = inputs.angleRads;
    }

    /*
    Experimental, might have the same results as other stop function.
     */
    public void stop(double xVelocity, double yVelocity) {
        inputs.setpointDriveVoltage = 0;
        inputs.setpointVelocityMetersPerSecond = 0;
        inputs.setpointAngleRads = Math.atan2(yVelocity, xVelocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (io.encoderJustConnected() && !io.initializedAngleFalcon()) {
            io.resetAngle();
        }
        io.configMotionMagic(getConfigFromDashboard());

        currentDrawCoulombs.update(inputs.appliedDriveCurrent + inputs.appliedAngleCurrent);
        inputs.totalCurrentDrawCoulombs = currentDrawCoulombs.get();

        Logger.getInstance().processInputs("Module_" + moduleNameOf(number), inputs);

        io.setDriveVoltage(inputs.setpointDriveVoltage);
        io.setAngle(inputs.angleRads + (inputs.setpointAngleRads - MathUtil.angleModulus(inputs.angleRads)));
    }

    public double[] getConfigFromDashboard() {
        return new double[] {
                kP.get(),
                kI.get(),
                kD.get(),
                kF.get(),
                sCurveStrength.get(),
                cruiseVelocity.get(),
                maxAcceleration.get(),
                allowableError.get(),
                maxIntegralAccumulator.get(),
                peakOutput.get()
        };
    }

    public static String moduleNameOf(int number) {
        if (number == 1) {
            return "FR";
        } else if (number == 2) {
            return "FL";
        } else if (number == 3) {
            return "RR";
        }
        return "RL";
    }

    public static Module of(int number, boolean isReal) {
        if (isReal) {
            return new Module(
                    new ModuleIOTalonFX(
                            DRIVE_IDS[number - 1], DRIVE_INVERTED[number - 1],
                            ANGLE_IDS[number - 1], ANGLE_INVERTED[number - 1],
                            ENCODER_IDS[number - 1], OFFSETS[number - 1]
                    ),
                    number, MOTION_MAGIC_CONFIGS[number - 1]);
        } else {
            return new Module(
                    new ModuleIOSim(),
                    number, MOTION_MAGIC_CONFIGS[number - 1]);
        }
    }
}
