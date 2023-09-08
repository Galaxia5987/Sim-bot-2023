package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Utils;
import frc.robot.utils.math.differential.Integral;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class Drive extends SubsystemBase {

    private static Drive INSTANCE = null;

    private final Module frontLeft;
    private final Module frontRight;
    private final Module rearLeft;
    private final Module rearRight;

    private final SwerveDriveKinematics kinematics;
    public Rotation2d Angle;
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] desiredModuleStates;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs;
    private final Integral yawSim;

    private SwerveModuleState[] currentModuleStates;
    private SwerveModulePosition[] currentModulePositions;
    private ChassisSpeeds currentSpeeds;
    private final SwerveDriveOdometry odometry;
    private final DriveIOInputsAutoLogged driveInputs;

    private double yawOffset = 0.0;
    private double currentYaw = 0.0;

    private Drive() {
        frontLeft = Module.of(1, Robot.isReal());
        frontRight = Module.of(2, Robot.isReal());
        rearLeft = Module.of(3, Robot.isReal());
        rearRight = Module.of(4, Robot.isReal());

        kinematics = new SwerveDriveKinematics(
                new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        desiredModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        currentModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        currentModulePositions = new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), currentModulePositions);

        if (Robot.isReal()) {
            gyroIO = new GyroIONavx();
            yawSim = null;
        } else {
            yawSim = new Integral(0, 0);
            gyroIO = new GyroIO() {};
        }

        gyroInputs = new GyroIOInputsAutoLogged();
        driveInputs = new DriveIOInputsAutoLogged();
    }

    public static Drive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Drive();
        }
        return INSTANCE;
    }

    public void drive(ChassisSpeeds speeds, boolean fieldOriented) {
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    new Rotation2d(gyroInputs.yawPositionRad)
            );
        }

        desiredSpeeds = speeds;
        desiredModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
    }

    public void drive(double xOutput, double yOutput, double omegaOutput, double deadband, boolean fieldOriented) {
        double magnitude = Math.sqrt(xOutput * xOutput + yOutput * yOutput);
        double angle = Math.atan2(yOutput, xOutput);
        magnitude = MathUtil.applyDeadband(magnitude, deadband);
        xOutput = magnitude * Math.cos(angle);
        yOutput = magnitude * Math.sin(angle);
        omegaOutput = MathUtil.applyDeadband(omegaOutput, 0.05);

        drive(new ChassisSpeeds(
                        xOutput * MAX_VELOCITY_METERS_PER_SECOND,
                        yOutput * MAX_VELOCITY_METERS_PER_SECOND,
                        omegaOutput * MAX_ROTATIONAL_VELOCITY),
                fieldOriented);
    }

    public void setDesiredModuleStates(SwerveModuleState[] desiredModuleStates) {
        this.desiredModuleStates = desiredModuleStates;
    }

    public ChassisSpeeds getCurrentVelocity() {
        return currentSpeeds;
    }

    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getCurrentAngle(){
        return getCurrentPose().getRotation();
    }

    public SwerveModulePosition[] getCurrentModulePositions() {
        return currentModulePositions;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(currentYaw), currentModulePositions, pose);
    }

    public void resetYaw(double yaw) {
        yawOffset = yaw - gyroInputs.rawYawPositionRad;
        currentYaw = gyroInputs.yawPositionRad + yawOffset;
    }

    public void resetOdometry(Pose2d pose, double yaw) {
        resetYaw(yaw);
        resetOdometry(pose);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        rearLeft.stop();
        rearRight.stop();
    }

    public void stop(double velocityX, double velocityY) {
        frontLeft.stop(velocityX, velocityY);
        frontRight.stop(velocityX, velocityY);
        rearLeft.stop(velocityX, velocityY);
        rearRight.stop(velocityX, velocityY);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            desiredSpeeds = new ChassisSpeeds();
            desiredModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
        }

        // Updating swerve module states
        currentModuleStates = new SwerveModuleState[]{
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState()
        };
        currentModulePositions = new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };
        currentSpeeds = kinematics.toChassisSpeeds(currentModuleStates);

        // Updating gyro inputs
        gyroIO.updateInputs(gyroInputs);

        if (Robot.isSimulation()) {
            yawSim.update(currentSpeeds.omegaRadiansPerSecond);

            gyroInputs.gyroConnected = true;
            gyroInputs.rawYawPositionRad = yawSim.get();
        }
        gyroInputs.yawPositionRad = gyroInputs.rawYawPositionRad + yawOffset;

        // Updating odometry and pose estimation
        odometry.update(new Rotation2d(gyroInputs.rawYawPositionRad), currentModulePositions);

        // Setting desired states
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, MAX_VELOCITY_METERS_PER_SECOND);

        if (Utils.speedsEpsilonEquals(desiredSpeeds)) {
            for (int i = 0; i < 4; i++) {
                desiredModuleStates[i] = new SwerveModuleState(0, currentModuleStates[i].angle);
            }
            desiredSpeeds = new ChassisSpeeds();
            stop();
        } else {
            frontLeft.set(desiredModuleStates[0]);
            frontRight.set(desiredModuleStates[1]);
            rearLeft.set(desiredModuleStates[2]);
            rearRight.set(desiredModuleStates[3]);
        }

        // Update drive inputs
        driveInputs.currentModuleStates = Utils.swerveModuleStatesToArray(currentModuleStates);
        driveInputs.currentSpeeds = Utils.chassisSpeedsToArray(currentSpeeds);
        driveInputs.desiredModuleStates = Utils.swerveModuleStatesToArray(desiredModuleStates);
        driveInputs.desiredSpeeds = Utils.chassisSpeedsToArray(desiredSpeeds);
        driveInputs.poseMeters = Utils.pose2dToArray(odometry.getPoseMeters());

        // Log inputs
        Logger.getInstance().processInputs("Drive", gyroInputs);
        Logger.getInstance().processInputs("Drive", driveInputs);
    }

    @AutoLog
    public static class DriveIOInputs {
        public double[] currentSpeeds;
        public double[] desiredSpeeds;
        public double[] currentModuleStates;
        public double[] desiredModuleStates;
        public double[] poseMeters;
    }
}
