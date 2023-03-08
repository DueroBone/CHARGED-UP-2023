package frc.robot.commands;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DeviceConstants;
import frc.robot.RobotContainer.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import java.lang.Math;

public class GoTele extends CommandBase {
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("**Started {0} ", this.getName()));
  }

  // private double teleLeft = 0;
  // private double teleRight = 0;
  private boolean drivingEnabled = true;
  private boolean armEnabled = true;
  private double deadzone = -1;
  private final DriveTrain drivetrain;
  private double speedMultiplier = 1;
  private double armUpMax = DeviceConstants.armUpMax;
  private double armDownMax = DeviceConstants.armDownMax;
  private double armInMax = DeviceConstants.armInMax;
  private double armOutMax = DeviceConstants.armOutMax;
  private double armDeadzone = -1;
  private int counter = 0;

  /**
   * Identifies active driving controller and activates drivetrain
   */
  public GoTele(boolean drivingEnabled, boolean armEnabled, double driveDeadzone, double topSpeed, double armDeadzone) {
    this.drivetrain = RobotContainer.m_driveTrain; // get driveTrain object from RobotContainer
    this.drivingEnabled = drivingEnabled;
    this.deadzone = driveDeadzone;
    this.speedMultiplier = topSpeed;
    this.armDeadzone = armDeadzone;
    this.armEnabled = armEnabled;
    this.counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  @Override
  public void execute() {
    if (counter == 0) {
      if (!drivingEnabled) {
        DriverStation.reportWarning("## Driving is disabled ##", false);
      }
      if (!armEnabled) {
        DriverStation.reportWarning("## Arm manual control is disabled ##", false);
      }
    }
    // double deadzone = 0.0;
    double teleLeft = 0;
    double teleRight = 0;
    double armLift = 0;
    double armExtend = 0;

    boolean usingConDynX = Math.abs(dynamicXbox.object.getLeftY()) > deadzone
        || Math.abs(dynamicXbox.object.getRightY()) > deadzone;
    boolean usingConDynP = Math.abs(dynamicPlaystation.object.getLeftY()) > deadzone
        || Math.abs(dynamicPlaystation.object.getRightY()) > deadzone;
    if (dynamicPlaystation.object.isConnected() && usingConDynP) {
      teleLeft = dynamicPlaystation.object.getLeftY() * -1;
      teleRight = dynamicPlaystation.object.getRightY() * -1;

      if (dynamicPlaystation.LeftTrigger.get() == true) {
        teleLeft = (dynamicPlaystation.object.getLeftY() + dynamicPlaystation.object.getRightY()) / (-2);
        teleRight = teleLeft;
      }
    } else {
      if (dynamicXbox.object.isConnected() && usingConDynX) {
        teleLeft = dynamicXbox.object.getLeftY() * -1;
        teleRight = dynamicXbox.object.getRightY() * -1;

        if (dynamicXbox.LeftTrigger.get() == true) {
          teleLeft = (dynamicXbox.object.getLeftY() +
              dynamicXbox.object.getRightY()) / (-2);
          teleRight = teleLeft;
        }
      }
    }

    if (dynamicJoystick.object.isConnected()) {
      armLift = dynamicJoystick.object.getY() * -1;
      armExtend = dynamicJoystick.object.getX();
    }

    double a = 1 - deadzone;
    a = 1 / a;

    if (Math.abs(teleLeft) > deadzone) {
      if (teleLeft > 0) {
        teleLeft = teleLeft - deadzone;
      } else {
        teleLeft = teleLeft + deadzone;
      }
      teleLeft = teleLeft * a;
      teleLeft = smartSquare(teleLeft, Constants.DriveConstants.drivingExponent);
      teleLeft = teleLeft * speedMultiplier;
    } else {
      teleLeft = 0;
    }
    if (Math.abs(teleRight) > deadzone) {
      if (teleRight > 0) {
        teleRight = teleRight - deadzone;
      } else {
        teleRight = teleRight + deadzone;
      }
      teleRight = teleRight * a;
      teleRight = smartSquare(teleRight, Constants.DriveConstants.drivingExponent);
      teleRight = teleRight * speedMultiplier;
    } else {
      teleRight = 0;
    }
    if (Math.abs(armLift) > armDeadzone) {
      if (armLift > 0) {
        armLift = armLift - armDeadzone;
      } else {
        armLift = armLift + armDeadzone;
      }
      armLift = armLift * a;
      armLift = smartSquare(armLift, Constants.DriveConstants.drivingExponent);
      armLift = armLift * speedMultiplier;
    } else {
      armLift = 0;
    }
    if (Math.abs(armExtend) > armDeadzone) {
      if (armExtend > 0) {
        armExtend = armExtend - armDeadzone;
      } else {
        armExtend = armExtend + armDeadzone;
      }
      armExtend = armExtend * a;
      armExtend = smartSquare(armExtend, Constants.DriveConstants.drivingExponent);
      armLift = armExtend * speedMultiplier;
    } else {
      armExtend = 0;
    }

    if (armLift != 0) {
      if (armLift > 0) {
        armLift = armLift * armUpMax;
      } else {
        armLift = armLift * armDownMax;
      }
    }
    if (armExtend != 0) {
      if (armExtend > 0) {
        armExtend = armExtend * armOutMax;
      } else {
        armExtend = armExtend * armInMax;
      }
    }

    if (drivingEnabled) {
      DriveTrain.doTankDrive(teleLeft, teleRight);
    }
    if (armEnabled) {
      Arm.setLifter(armLift);
      Arm.setExtender(armExtend);
    }
    SmartDashboard.putNumber("Left Drive Speed", teleLeft);
    SmartDashboard.putNumber("Right Drive Speed", teleRight);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double smartSquare(double input, int exponent) {
    double output = Math.pow(input, exponent);
    if (Math.signum(input) != Math.signum(output)) {
      output = output * -1;
    }
    return output;
  }
}
