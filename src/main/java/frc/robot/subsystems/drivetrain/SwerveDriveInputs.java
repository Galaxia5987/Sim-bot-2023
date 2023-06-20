package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveDriveInputs {
    public double supplyCurrent;
    public double statorCurrent;

    public double[] currentSpeeds ={0, 0, 0};
    public double[] desiredSpeeds ={0, 0, 0};

}
