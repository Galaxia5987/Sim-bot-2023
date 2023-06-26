package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveDriveInputs {
    public double supplyCurrent;
    public double statorCurrent;

    public double[] currentSpeeds ={0, 0, 0};
    public double[] desiredSpeeds ={0, 0, 0};

    public double[] absolutePositions =new double[4];

    public double rawYaw;
    public double yaw;
    public double gyroOffset;

}
