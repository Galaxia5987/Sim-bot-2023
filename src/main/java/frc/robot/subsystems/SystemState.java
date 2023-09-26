package frc.robot.subsystems;

public interface SystemState<T extends SystemState<T, Inputs>, Inputs> {

    T nextState(Inputs inputs);
}
