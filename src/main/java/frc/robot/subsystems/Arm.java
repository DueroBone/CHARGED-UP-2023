package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DeviceConstants;;

public class Arm {
  static String preset = "starting";
  static double actualHeight;
  static double actualLength;
  static double toleranceHeight = 5;
  static double toleranceLength = 5;
  static CANSparkMax lifterMotor = new CANSparkMax(DeviceConstants.armLifterId, MotorType.kBrushless);
  static CANSparkMax extenderMotor = new CANSparkMax(DeviceConstants.armExtenderId, MotorType.kBrushless);

  public static void setup() {
    lifterMotor.setIdleMode(IdleMode.kBrake);
    lifterMotor.setSmartCurrentLimit(10);
    extenderMotor.setIdleMode(IdleMode.kBrake);
    extenderMotor.setSmartCurrentLimit(10);
  }

  public static void moveArmToPreset(String postition, double lifterSpeed, double extenderSpeed) {
    preset = postition;
    double desiredHeight = 0;
    double desiredLength = 0;
    actualHeight = 0; // FIND ACTUAL POSITIONS
    actualLength = 0;

    if (preset == "starting") {
      desiredHeight = positions.startingHeight;
      desiredLength = positions.startingLength;
    } else if (preset == "driving") {
      desiredHeight = positions.drivingHeight;
      desiredLength = positions.drivingLength;
    } else if (preset == "bottom") {
      desiredHeight = positions.bottomHeight;
      desiredLength = positions.bottomLength;
    } else if (preset == "scoring") {
      desiredHeight = positions.scoringHeight;
      desiredLength = positions.scoringLength;
    } else {
      desiredHeight = positions.startingHeight;
      desiredLength = positions.startingLength;
    }

    if (actualHeight < desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        setLifter(lifterSpeed);
      } else {
        stopLifter();
      }
    } else if (actualHeight > desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        setLifter(-lifterSpeed);
      } else {
        stopLifter();
      }
    } else {
      stopLifter();
    }

    if (actualLength < desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        setExtender(extenderSpeed);
      } else {
        stopExtender();
      }
    } else if (actualLength > desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        setExtender(-extenderSpeed);
      } else {
        stopExtender();
      }
    } else {
      stopExtender();
    }
  }

  public static void setLifter(double speed) {
    lifterMotor.set(speed);
  }

  public static void setExtender(double speed) {
    extenderMotor.set(speed);
  }

  public static void stopLifter() {
    lifterMotor.stopMotor();
  }

  public static void stopExtender() {
    extenderMotor.stopMotor();
  }

  public static void stopArm() {
    lifterMotor.stopMotor();
    extenderMotor.stopMotor();
  }

  public static void statrtingPosition() {
    preset = "starting";
  }

  public static void drivingPosition() {
    preset = "driving";
  }

  public static void bottomPosition() {
    preset = "bottom";
  }

  public static void scoringPosition() {
    preset = "scoring";
  }
}

class positions {
  public static double startingHeight = 0;
  public static double startingLength = 0;

  public static double drivingHeight = 0;
  public static double drivingLength = 0;

  public static double bottomHeight = 0;
  public static double bottomLength = 0;

  public static double scoringHeight = 0;
  public static double scoringLength = 0;
}