package frc.robot.autonomous;

import java.util.ArrayList;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionPipeline;

public class AutoSpinToAngleTarget extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final VisionPipeline m_visionPipeline;
  private double targetAngle;
  private double turnPower;

  private PIDController pid;
  private static final double kP = 0.03; // P term constant (Proportional)
  private static final double kI = 0.00; // I term constant (Integral)
  private static final double kD = 0.00; // D term constant (Derivative)
  // private static final double kF = 0.0001; // F term constant (Feedforward)

  private static double currentHeading;
  private double currentDiff;
  private double speed = 0.0;
  private double slowDownReducer = 0.5; // Power reduction amount when near target || Was 85
  private boolean inRange = false;
  private int counter1 = 2;
  private int counter2 = 2;
  private int counter3 = 2;
  private double startingHeading;

  private static double tolerance = 3;

  private double FindObject() {
    ArrayList<MatOfPoint> startingArray = m_visionPipeline.filterContoursOutput();

    Point target = new Point(120.0, 120.0);
    MatOfPoint closestContour = startingArray.get(0);
    double closestDistance = Double.MAX_VALUE;

    for (MatOfPoint contour : startingArray) {
      Rect rect = Imgproc.boundingRect(contour);
      Point center = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
      double distance = Math.sqrt(Math.pow(center.x - target.x, 2) + Math.pow(center.y - target.y, 2));
      if (distance < closestDistance) {
        closestContour = contour;
        closestDistance = distance;
      }
    }

    Rect rect = Imgproc.boundingRect(closestContour);
    double centerX = rect.x + rect.width / 2.0;
    centerX = centerX-(Robot.CamWidth/2);
    centerX = centerX/(Robot.CamWidth/2);
    centerX = centerX*180;
    return centerX;
  }

  /**
   * CRASHES THE ROBOT IMMEDIATELY
   */
  public AutoSpinToAngleTarget(double turnPower) {
    this.m_driveTrain = RobotContainer.m_driveTrain;
    this.m_visionPipeline = VisionPipeline.m_visionPipeline;
    this.turnPower = turnPower;
    this.pid = new PIDController(kP, kI, kD);
    addRequirements(this.m_driveTrain);
    DriverStation.reportError("Crashing Robot on command", false);
  }

  @Override
  public void initialize() {
    DriveTrain.stop();
    // Start Vision
    startingHeading = FindObject();
    pid.reset();
    pid.setSetpoint(targetAngle);
    pid.setTolerance(tolerance);
    pid.enableContinuousInput(-180.0, 180.0); // Enable continuous input in range from -180 to 180
    System.out.println("**starting AutoSpinToAnglePID target angle: " + targetAngle + " heading: " + 0);
  }

  @Override
  public void execute() {
    targetAngle = FindObject();
    currentHeading = startingHeading - targetAngle;
    currentDiff = targetAngle - currentHeading;
    inRange = (Math.abs(currentDiff) <= tolerance);

    if (inRange) {
      DriveTrain.stop();
      System.out.println("**in range stop turn - heading: " + String.format("%.3f", currentHeading));
    } else {

      // clamp pid output to between -turnPower and +turnPower
      double pidOutput = MathUtil.clamp(pid.calculate(currentHeading, targetAngle), -turnPower, turnPower);
      if (counter1++ % 3 == 0) {
        System.out.println("**diff: " + currentDiff + " heading: "
            + currentHeading + " target: " + targetAngle + " inRange: " + inRange);
      }

      if (pidOutput >= -0.5 && pidOutput <= 0.5) {
        speed = turnPower * slowDownReducer; // slow down if close to target
      } else {
        speed = turnPower; // Otherwise do speed requested
      }
      if (pidOutput <= 0) {
        if (counter2++ % 3 == 0) {
          System.out.println("**turn Left Correction pidOut: " + String.format("%.3f  ", pidOutput) + " heading: " + currentHeading);
        }
        DriveTrain.doTankDrive(-speed, speed); // Turn to left ( reverse left side)
      } else {

        if (counter3++ % 3 == 0) {
          System.out.println("**turn Right Correction pidOut: " + String.format("%.3f  ", pidOutput) + " heading: " + currentHeading);
        }
        DriveTrain.doTankDrive(speed, -speed); // turn to right ( reverse right side)
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    DriveTrain.stop();
    System.out
        .println("**ending AutoSpinToAnglePID command  current heading: " + String.format("%.3f", currentHeading));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inRange;
  }
}
