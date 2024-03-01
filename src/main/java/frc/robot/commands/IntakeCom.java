// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class IntakeCom extends Command {

  IntakeSub intakee;
  boolean lower;
  boolean raise;
  boolean takeIn;
  boolean all;

  /** Creates a new sensorCom. */
  public IntakeCom(IntakeSub intakey, boolean down, boolean up, boolean take, boolean allOf) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakee = intakey;
    lower = down;
    raise = up;
    take = takeIn;
    all = allOf;

    addRequirements(intakee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lower){
      intakee.extendIntake();
    }
    if(takeIn){
      intakee.takeNode();
    }
    if(raise){
      intakee.retractIntake();
    }
    if(all){
      intakee.extendIntake();
      intakee.takeNode();
      intakee.retractIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
