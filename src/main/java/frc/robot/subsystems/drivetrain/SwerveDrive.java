package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.utils.Utils;
import org.eclipse.jetty.util.MathUtils;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive INSTANCE = null;
    private SwerveDriveInputsAutoLogged loggerInputs = new SwerveDriveInputsAutoLogged();

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveModule[] modules = new SwerveModule[4]; //FL, FR, RL, RR
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];
    private SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            SwerveConstants.wheelPositions[0],
            SwerveConstants.wheelPositions[1],
            SwerveConstants.wheelPositions[2],
            SwerveConstants.wheelPositions[3]);

    private double gyroOffset = 0;

    private SwerveDrive() {
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(Ports.SwerveDrive.DRIVE_IDS[i], Ports.SwerveDrive.ANGLE_IDS[i],
                    Ports.SwerveDrive.ENCODER_IDS[i], SwerveConstants.motionMagicConfigs[i],i+1);
        }
    }

    public static SwerveDrive getInstance(){
        if (INSTANCE==null){
            INSTANCE = new SwerveDrive();
        }
        return INSTANCE;
    }

    /**
     * Updates the offset for the gyro.
     * @param angle The desired angle. [rad]
     */
    public void resetGyro(double angle){
        gyroOffset = angle - getRawYaw();
        loggerInputs.gyroOffset = gyroOffset;
    }

    public void resetGyro(){
        resetGyro(0);
    }

    /**
     * Gets the raw yaw reading from the gyro.
     * @return Yaw angle reading from gyro. [rad]
     */
    public double getRawYaw(){
        return MathUtil.angleModulus(Math.toRadians(gyro.getAngle())); //TODO: check if this fixed the issue
    }

    /**
     * Gets the yaw reading from the gyro with the calculated offset.
     * @return Yaw angle with offset. [rad]
     */
    public double getYaw(){
        return getRawYaw() + gyroOffset;
    }

    /**
     * Sets the module states to the desired module states.
     * @param desiredModuleStates The desired module states to set the modules to.
     */
    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, SwerveConstants.MAX_X_Y_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setModuleState(desiredModuleStates[i]);
        }
    }

    /**
     * Updates each module position with an offset and an absolute encoder.
     * @param offsets Offsets for each of the modules. [sensor ticks]
     */
    public void updateOffsets(double[] offsets){
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateOffset(offsets[i]);
        }
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
        loggerInputs.desiredSpeeds = Utils.chassisSpeedsToArray(chassisSpeeds);

//        if (fieldOriented){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    chassisSpeeds.omegaRadiansPerSecond,
                    new Rotation2d(getYaw())
            );
            System.out.println("fieldOriented "+chassisSpeeds.toString());
//        }

        System.out.println("robot oreiented "+chassisSpeeds.toString());
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the swerve
     *
     * @param xOutput     percentage of the x speed
     * @param yOutput     percentage of the y speed
     * @param omegaOutput percentage of the omega speed
     */
    public void drive(double xOutput, double yOutput, double omegaOutput, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput);

        drive(chassisSpeeds, fieldOriented);
    }

    public void periodic(){
        for (int i=0; i< modules.length; i++){
            currentModuleStates[i] = modules[i].getModuleState();
//            loggerInputs.currentSpeeds[i] = modules[i].getSpeed(); //TODO: check if this works
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
        }

        loggerInputs.supplyCurrent =
                modules[0].getSupplyCurrent()+modules[1].getSupplyCurrent()+modules[2].getSupplyCurrent()+modules[3].getStatorCurrent();

        loggerInputs.statorCurrent =
                modules[0].getStatorCurrent()+modules[1].getStatorCurrent()+modules[2].getStatorCurrent()+modules[3].getStatorCurrent();

        loggerInputs.rawYaw = getRawYaw();
        loggerInputs.yaw = getYaw();

        Logger.getInstance().processInputs("SwerveDrive", loggerInputs);
    }
}
