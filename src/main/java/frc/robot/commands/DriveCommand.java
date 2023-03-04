// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_leftSupplier; // button value for left power
  private final DoubleSupplier m_rightSupplier; // button value for right power

  // private final double kDeadbandRange = 0.02;
  // private int counter1 = 40; // for limiting display

  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(DoubleSupplier leftPower, DoubleSupplier rightPower) {
    this.m_driveTrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.m_leftSupplier = leftPower;
    this.m_rightSupplier = rightPower;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Drive robot with tank drive using controller values passed in
    // Controller axis are measured from -1.0 to 1.0. -1.0 being all way down and
    // +1.0 being all way up
    // Motor controllers also take values from -1.0 to 1.0. -1.0 being 100% power
    // backwards, +1 100^ forward
    // Y axis of left stick moves the left side of the robot forward and backward
    // Y axis of right stick moves the right side of the robot forward and backward

    double leftDrivePower = m_leftSupplier.getAsDouble(); // get value of DoubleSupplier passed in
    double rightDrivePower = m_rightSupplier.getAsDouble();

    // ** by default WPILib Differential Drive class applies an input deadband of
    // 0.02
    // Check if in deadband range of joysticks in which robot should not respond
    // if (Math.abs(leftDrivePower) < kDeadbandRange) { leftDrivePower = 0; }
    // if (Math.abs(rightDrivePower) < kDeadbandRange) { rightDrivePower = 0; }
    DriveTrain.doTankDrive(leftDrivePower, rightDrivePower);
    // if (counter1++ % 40 == 0) {
    // System.out.println(" Drive leftPower "+leftDrivePower + " rightPower
    // "+rightDrivePower);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // this command should not end
  }
}
