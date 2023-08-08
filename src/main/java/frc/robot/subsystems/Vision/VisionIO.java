package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {


    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputs inputs);

    default Pose3d AprilChooser(int aprilID){
        Pose3d[] poses;
        if (Robot.isReal()){
            poses = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d()) }; // TODO: change for the tent's field
        }
        else{
            poses = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d()) };
        }
        return poses[aprilID];
    }

    default Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented, int aprilId) {
        Transform3d transform3d = new Transform3d(poseTargetOriented.getTranslation(), poseTargetOriented.getRotation());
        return AprilChooser(aprilId).plus(transform3d);
    }

    @AutoLog
    class VisionInputs {
        double latency = 0;
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        int targetID = 0;
        double[] poseTargetOriented = new double[7];
        double[] poseFieldOriented = new double[7];
        Pose3d poseTargetOriented3d = new Pose3d();
        Pose3d poseFieldOriented3d = new Pose3d();

    }
}
