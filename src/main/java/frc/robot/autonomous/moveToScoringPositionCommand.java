// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class moveToScoringPositionCommand extends CommandBase {
  /** Creates a new moveToScoringPosition. */
  public moveToScoringPositionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.scoringPosition();
    Arm.moveToPreset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.moveToPreset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(Arm.info.getLifterSpeed()) <= 0.1) && (Math.abs(Arm.info.getExtenderSpeed()) <= 0.1);
  }
}
