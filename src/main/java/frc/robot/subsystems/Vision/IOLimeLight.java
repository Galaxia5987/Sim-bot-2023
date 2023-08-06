package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.*;

public class IOLimeLight implements VisionIO {

    private final AprilTagFieldLayout fieldLayout;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public IOLimeLight(int ip) {
        PortForwarder.add(5800, "10.59.87." + ip, 5800);
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
    public void updateInputs(VisionInputs inputs) {
        inputs.yaw = table.getDoubleTopic("tx").getEntry(0.0).get();

        inputs.pitch = table.getDoubleTopic("ty").getEntry(0.0).get();

        inputs.area = table.getDoubleTopic("ta").getEntry(0.0).get();

        inputs.latency = table.getDoubleTopic("tl").subscribe(0.0).get(); //Todo: add latency to IOPhoton

        inputs.targetID = (int) table.getDoubleTopic("tid").getEntry(0.0).get();

        if((table.getDoubleTopic("tv").subscribe(0.0).get()) == 1){inputs.hasTargets = true;}
        inputs.targetSkew = 0;

        double[] targetRobotOriented = table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[6]).get();
        inputs.poseTargetOriented3d = new Pose3d(targetRobotOriented[0], targetRobotOriented[1], targetRobotOriented[2], new Rotation3d(targetRobotOriented[3], targetRobotOriented[4], targetRobotOriented[5]));
        inputs.poseFieldOriented3d = getEstimatedPoseFieldOriented(inputs.poseTargetOriented3d, inputs.targetID);

        inputs.poseTargetOriented = new double[]{
                inputs.poseTargetOriented3d.getX(),
                inputs.poseTargetOriented3d.getY(),
                inputs.poseTargetOriented3d.getZ(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getX(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getY(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getZ(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getW()};
        inputs.poseFieldOriented = new double[]{
                inputs.poseFieldOriented3d.getX(),
                inputs.poseFieldOriented3d.getY(),
                inputs.poseFieldOriented3d.getZ(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getX(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getY(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getZ(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getW()};
    }
}
