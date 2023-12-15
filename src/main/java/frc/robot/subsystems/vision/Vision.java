package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import frc.robot.swerve.SwerveDrive;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final VisionModule[] modules;
    private final Result[] results;

    private Vision(VisionModule... modules) {
        this.modules = modules;
        results = new Result[modules.length];
    }

    public static Vision getInstance() {
        if (INSTANCE == null) {
            if (Robot.isReal()) {
                INSTANCE = new Vision(
                        VisionModule.photonVisionIO("LeftPie", 0),
                        VisionModule.photonVisionIO("RightPie", 1)
                );
            } else {
                INSTANCE = new Vision(
                        VisionModule.simIO("simCam", 0)
                );
            }
        }
        return INSTANCE;
    }

    public Result[] getResults() {
        return results;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            module.io.updateInputs(module.inputs);
            Logger.processInputs(module.name, module.inputs);
            results[i] = module.io.getLatestResult();
            Logger.recordOutput("RobotToCamLeft", new Pose3d(SwerveDrive.getInstance().getBotPose()).plus(VisionConstants.ROBOT_TO_CAM[0]));
        }
    }
}
