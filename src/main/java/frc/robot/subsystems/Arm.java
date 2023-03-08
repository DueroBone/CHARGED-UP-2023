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
  static double armUpMax = DeviceConstants.armUpMax;
  static double armDownMax = DeviceConstants.armDownMax;
  static double armInMax = DeviceConstants.armInMax;
  static double armOutMax = DeviceConstants.armOutMax;
  static CANSparkMax lifterMotor = new CANSparkMax(DeviceConstants.armLifterId, MotorType.kBrushless);
  static CANSparkMax extenderMotor = new CANSparkMax(DeviceConstants.armExtenderId, MotorType.kBrushless);

  public static void setup() {
    lifterMotor.setIdleMode(IdleMode.kBrake);
    lifterMotor.setSmartCurrentLimit(10);
    lifterMotor.setInverted(true);
    extenderMotor.setIdleMode(IdleMode.kBrake);
    extenderMotor.setSmartCurrentLimit(10);
  }

  public static void moveArmToPreset() {
    double desiredHeight = 0;
    double desiredLength = 0;
    getPositions(); // FIND ACTUAL POSITIONS

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
        moveLifter(false);
      } else {
        stopLifter();
      }
    } else if (actualHeight > desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        moveLifter(true);
      } else {
        stopLifter();
      }
    } else {
      stopLifter();
    }

    if (actualLength < desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        moveExtender(true);
      } else {
        stopExtender();
      }
    } else if (actualLength > desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        moveExtender(false);
      } else {
        stopExtender();
      }
    } else {
      stopExtender();
    }
  }

  public static void moveLifter(boolean up) {
    if (up) {
      setLifter(armUpMax);
    } else {
      setLifter(armDownMax);
    }
  }

  public static void moveExtender(boolean out) {
    if (out) {
      setExtender(armOutMax);
    } else {
      setExtender(armInMax);
    }
  }

  public static void setLifter(double speed) {
    lifterMotor.set(speed);
    //System.out.println("Lifter set to " + speed);
  }

  public static void setExtender(double speed) {
    extenderMotor.set(speed);
    //System.out.println("Extender set to " + speed);
  }

  public static void stopLifter() {
    lifterMotor.stopMotor();
    //System.out.println("Stopping lifter");
  }

  public static void stopExtender() {
    extenderMotor.stopMotor();
    //System.out.println("Stopping lifter");
  }

  public static void stopArm() {
    lifterMotor.stopMotor();
    extenderMotor.stopMotor();
  }

  public static void startingPosition() {
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

  static void getPositions() {
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