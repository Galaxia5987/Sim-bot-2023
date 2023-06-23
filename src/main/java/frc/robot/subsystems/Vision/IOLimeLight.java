package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.*;

public class IOLimeLight implements VisionIO {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber tx = table.getDoubleTopic("tx").subscribe(0.0);
    private final DoubleSubscriber ty = table.getDoubleTopic("ty").subscribe(0.0);
    private final IntegerSubscriber tv = table.getIntegerTopic("tv").subscribe(0);
    private final DoubleSubscriber ts = table.getDoubleTopic("ts").subscribe(0.0);
    private final IntegerSubscriber tid = table.getIntegerTopic("tid").subscribe(0);
    private final IntegerSubscriber getpipe = table.getIntegerTopic("getpipe").subscribe(0);
    private final DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[6]);
    private final DoubleArraySubscriber botPoseFieldOriented = table.getDoubleArrayTopic("botpose").subscribe(new double[6]);
    private AprilTagFieldLayout fieldLayout;

    public IOLimeLight() {
        PortForwarder.add(5800, "photonvision.local", 5800);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }


    }

    @Override
    public Pose3d getEstimatedPose() {
        return null;
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {

    }

    @Override
    public void updateInputs(VisionInputsAutoLogged inputs) {
        inputs.yaw = table.getDoubleTopic("tx").subscribe(0.0).get();
        inputs.pitch = table.getDoubleTopic("ty").subscribe(0.0).get();
        inputs.area = table.getDoubleTopic("ta").subscribe(0.0).get();
        inputs.latency = table.getDoubleTopic("tl").subscribe(0.0).get(); //Todo: add latency to IOPhoton
        inputs.targetID = (int) table.getDoubleTopic("tid").subscribe(0.0).get();


    }
}
