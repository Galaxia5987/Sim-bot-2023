//package frc.robot.commandgroups;
//
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.subsystems.arm.Arm;
////import frc.robot.subsystems.arm.ArmPosition;
//
//public class GetArmIntoRobot extends SequentialCommandGroup {
//
//    public GetArmIntoRobot() {
//        Arm arm = Arm.getInstance();
//
//        addCommands(
//                new ArmWithSpline(ArmPosition.PICKUP)
////                new ArmAxisControl(1, 0, -0.02,
////                        0, 0).until(() -> arm.getEndPosition().getY() <= -0.13)
//        );
//    }
//}
