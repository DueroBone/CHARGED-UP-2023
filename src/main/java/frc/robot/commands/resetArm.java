// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class resetArm extends CommandBase {
  static int counter = 0;

  /** Creates a new resetArm. */
  public resetArm() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Arm.info.getLifterLimitUp()) {
      Arm.setLifter(Constants.DeviceConstants.armUpMax / 3);
    }
    if (!Arm.info.getExtenderLimitIn()) {
      Arm.setExtender(Constants.DeviceConstants.armInMax / 3);
    }
    if (counter++%5 == 0) {
      System.out.println("*Arm resetting*");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.info.resetEncoders();
    System.out.println("Arm reset finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Arm.info.getLifterLimitUp() && Arm.info.getExtenderLimitIn());
  }
}
