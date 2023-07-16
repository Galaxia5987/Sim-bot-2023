package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.drivetrain.Drive;
import org.photonvision.SimVisionSystem;

public class IOCameraSim implements VisionIO{
    SimVisionSystem simVisionSystem;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("simCamera");
    private final DoubleSubscriber yaw = table.getDoubleTopic("targetYaw").subscribe(0);
    private final DoubleSubscriber pitch = table.getDoubleTopic("targetPitch").subscribe(0);
    private final DoubleArrayTopic targetPose = table.getDoubleArrayTopic("targetPose").getEntry(new double[6]).get();

    public IOCameraSim(){
        simVisionSystem = new SimVisionSystem(
                "simCamera",
                90.0,
                VisionConstants.ROBOT_TO_CAM,
                7,
                1600,
                1200,
                0.0
        );
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
         simVisionSystem.processFrame(Drive.getInstance().getCurrentPose());
         targetPose.get()
    }

}
