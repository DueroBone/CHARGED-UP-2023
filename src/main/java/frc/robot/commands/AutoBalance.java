// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import java.lang.Math;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveTrain m_driveTrain;
  int balancingStage;
  int counter1;
  double speed = 0.1;
  int timeBalanced;
  int timeRequired = 10000;

  public AutoBalance(Boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    // m_driveTrain = new DriveTrain();
    addRequirements(m_driveTrain);
    if (!forward) {
      speed = speed * -1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.resetGyro();
    balancingStage = 0;
    counter1 = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move Forward until gyro gets past a threshold
    if (counter1 % 5 == 0) { // to slow it down
      DriverStation.reportError("Gyro at " + DriveTrain.m_Gyro.getPitch(), false);
      if (balancingStage == 0) {
        DriverStation.reportWarning("Started balance " + balancingStage, false);
        if (Math.abs(DriveTrain.m_Gyro.getPitch() + 90) > 3) {
          DriveTrain.doTankDrive(speed, speed);
        } else {
          balancingStage = 1;
          DriverStation.reportError("Moving to second stage", false);
        }

      } else if (balancingStage == 1) {
        DriveTrain.motorDriveLeft1.setIdleMode(IdleMode.kBrake); // Set brake mode
        DriveTrain.motorDriveLeft2.setIdleMode(IdleMode.kBrake);
        DriveTrain.motorDriveRight1.setIdleMode(IdleMode.kBrake);
        DriveTrain.motorDriveRight2.setIdleMode(IdleMode.kBrake);
        DriverStation.reportWarning("Next stage balance " + balancingStage, false);
        if (Math.abs(DriveTrain.m_Gyro.getPitch() + 90) > 3) {
          DriveTrain.doTankDrive(-speed / 2, -speed / 2);
        } else {
          balancingStage = 2;
          DriverStation.reportError("Moving to third stage", false);
        }
      } else if (balancingStage == 2) {
        timeBalanced = 0;
        
        while (timeBalanced < timeRequired) {
          while (Math.abs(DriveTrain.m_Gyro.getPitch() + 90) > 3) {
            if (DriveTrain.m_Gyro.getPitch() < 90) { // is even
              DriveTrain.doTankDrive(speed / 2, speed / 2);
            } else {
              DriveTrain.doTankDrive(-speed / 2, -speed / 2);
            }
          }
          timeBalanced++;
          DriverStation.reportError("Moving to another stage", false);
          while (System.currentTimeMillis() % 10 == 0) {
          }
        }
        DriverStation.reportWarning("Finished balance " + balancingStage, false);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    DriveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeBalanced >= timeRequired);
  }
}
