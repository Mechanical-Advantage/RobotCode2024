// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2024.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;

public class SteppableCommandGroup extends Command {
  private final Trigger forwardTrigger;
  private final Trigger reverseTrigger;
  private final List<Command> commands = new ArrayList<>();
  private boolean forwardTriggerState = false;
  private boolean reverseTriggerState = false;
  private int currentCommandIndex = -1;
  private boolean commandFinished = false;
  private boolean runsWhenDisabled = true;
  private InterruptionBehavior interruptionBehavior = InterruptionBehavior.kCancelIncoming;

  public SteppableCommandGroup(
      Trigger forwardTrigger, Trigger reverseTrigger, Command... commands) {
    this.forwardTrigger = forwardTrigger;
    this.reverseTrigger = reverseTrigger;
    addCommands(commands);
  }

  public final void addCommands(Command... commands) {
    if (currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      this.commands.add(command);
      m_requirements.addAll(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        interruptionBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    currentCommandIndex = 0;
    commandFinished = false;
    if (!commands.isEmpty()) {
      commands.get(0).initialize();
    }

    forwardTriggerState = forwardTrigger.getAsBoolean();
    reverseTriggerState = reverseTrigger.getAsBoolean();
  }

  @Override
  public final void execute() {
    if (commands.isEmpty()) {
      return;
    }

    Command currentCommand = commands.get(currentCommandIndex);
    if (currentCommand.isFinished() && !commandFinished) {
      currentCommand.end(false);
      commandFinished = true;
    }
    if (!commandFinished) {
      currentCommand.execute();
    }

    boolean stepForward = forwardTrigger.getAsBoolean() && !forwardTriggerState;
    boolean stepReverse = reverseTrigger.getAsBoolean() && !reverseTriggerState;
    forwardTriggerState = forwardTrigger.getAsBoolean();
    reverseTriggerState = reverseTrigger.getAsBoolean();

    int step = stepForward ? 1 : (stepReverse ? -1 : 0);
    int newCommandIndex = MathUtil.clamp(currentCommandIndex + step, 0, commands.size() - 1);
    if (newCommandIndex != currentCommandIndex) {
      if (!commandFinished) {
        currentCommand.end(true);
      }
      commands.get(newCommandIndex).initialize();
      currentCommandIndex = newCommandIndex;
      commandFinished = false;
    }
  }

  @Override
  public final void end(boolean interrupted) {
    if (interrupted && !commandFinished && !commands.isEmpty() && currentCommandIndex > -1) {
      commands.get(currentCommandIndex).end(true);
    }
    currentCommandIndex = -1;
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return interruptionBehavior;
  }

  public OptionalInt getCurrentCommandIndex() {
    if (currentCommandIndex == -1) {
      return OptionalInt.empty();
    } else {
      return OptionalInt.of(currentCommandIndex);
    }
  }
}
