package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.*;

public class IOLimeLight implements VisionIO {

    //private final DoubleSubscriber tx = table.getDoubleTopic("tx").subscribe(0.0);
    //private final DoubleSubscriber ty = table.getDoubleTopic("ty").subscribe(0.0);
    //private final IntegerSubscriber tv = table.getIntegerTopic("tv").subscribe(0);
    //private final DoubleSubscriber ts = table.getDoubleTopic("ts").subscribe(0.0);
    //private final IntegerSubscriber tid = table.getIntegerTopic("tid").subscribe(0);
    //private final IntegerSubscriber getpipe = table.getIntegerTopic("getpipe").subscribe(0);
    //private final DoubleArraySubscriber botPose = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[6]);
    //private final DoubleArraySubscriber botPoseFieldOriented = table.getDoubleArrayTopic("botpose").subscribe(new double[6]);
    private AprilTagFieldLayout fieldLayout;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public IOLimeLight() {
        PortForwarder.add(5800, "photonvision.local", 5800);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }


    }


    @Override
    public void setPipeLine(int pipeLineIndex) {

    }

    @Override
    public void updateInputs(VisionInputsAutoLogged inputs) {
        inputs.yaw = table.getDoubleTopic("tx").getEntry(0.0).get();

        inputs.pitch = table.getDoubleTopic("ty").getEntry(0.0).get();

        inputs.area = table.getDoubleTopic("ta").getEntry(0.0).get();

        inputs.latency = table.getDoubleTopic("tl").subscribe(0.0).get(); //Todo: add latency to IOPhoton

        inputs.targetID = (int) table.getDoubleTopic("tid").getEntry(0.0).get();

        inputs.poseFieldOriented = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[6]).get();

        if((table.getDoubleTopic("tv").subscribe(0.0).get()) == 1){inputs.hasTargets = true;}
        inputs.targetSkew = 0;

        double[] poseFieldArray = table.getDoubleArrayTopic("botpose").getEntry(new double[6]).get();
        var estimatedPoseField = new Pose3d(poseFieldArray[0], poseFieldArray[1], poseFieldArray[2],new Rotation3d(poseFieldArray[3], poseFieldArray[4], poseFieldArray[5]));
        inputs.poseFieldOriented = new double[]{estimatedPoseField.getX(), estimatedPoseField.getY(), estimatedPoseField.getZ(), estimatedPoseField.getRotation().getQuaternion().getX(), estimatedPoseField.getRotation().getQuaternion().getY(), estimatedPoseField.getRotation().getQuaternion().getZ(), estimatedPoseField.getRotation().getQuaternion().getW()};

        double[] poseTargetArray = table.getDoubleArrayTopic("botpose").getEntry(new double[6]).get();
        var estimatedPoseTarget = new Pose3d(poseTargetArray[0], poseTargetArray[1], poseTargetArray[2],new Rotation3d(poseTargetArray[3], poseTargetArray[4], poseTargetArray[5]));
        inputs.poseFieldOriented = new double[]{estimatedPoseTarget.getX(), estimatedPoseTarget.getY(), estimatedPoseTarget.getZ(), estimatedPoseTarget.getRotation().getQuaternion().getX(), estimatedPoseTarget.getRotation().getQuaternion().getY(), estimatedPoseTarget.getRotation().getQuaternion().getZ(), estimatedPoseTarget.getRotation().getQuaternion().getW()};

    }
}
