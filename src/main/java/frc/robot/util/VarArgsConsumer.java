package frc.robot.util;

@FunctionalInterface
public interface VarArgsConsumer<T> {
  void accept(T... args);
}
