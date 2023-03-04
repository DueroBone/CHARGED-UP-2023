// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.AutoSpinToAngleTarget;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.GoTele;
import frc.robot.subsystems.VisionLight;
import frc.robot.subsystems.ControllerTracking;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Piston;
import frc.robot.subsystems.TestMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveTrain m_driveTrain = new DriveTrain();

  Compressor c = new Compressor(5, PneumaticsModuleType.REVPH);
  public static final Piston m_piston = new Piston();

  public static class PortBoundControllers {
    public static class PortZero {
      public static class Xbox {
        public static XboxController object = new XboxController(1);
        public static JoystickButton A;
        public static JoystickButton B;
        public static JoystickButton X;
        public static JoystickButton Y;
        public static JoystickButton LeftBumper;
        public static JoystickButton RightBumper;
        public static JoystickButton LeftStickPress;
        public static JoystickButton RightStickPress;
        public static JoystickButton Share;
        public static JoystickButton Options;
        static BooleanSupplier bSupplier = () -> false;
        public static Button LeftTrigger = new Button(bSupplier);
        public static Button RightTrigger = new Button(bSupplier);

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          System.out.println("Assigning static Xbox: " + object.getPort());
          A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
          B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
          X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
          Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
          LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
          RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
          LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
          RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
          Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
          Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
        }
      }

      public static class FlightStick {
        public static Joystick object = new Joystick(1);
        public static JoystickButton Trigger;
        public static JoystickButton Two;
        public static JoystickButton Three;
        public static JoystickButton Four;
        public static JoystickButton Five;
        public static JoystickButton Six;
        public static JoystickButton Seven;
        public static JoystickButton Eight;
        public static JoystickButton Nine;
        public static JoystickButton Ten;
        public static JoystickButton Eleven;

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          assignButtons();
        }

        public static void assignButtons() {
          System.out.println("Assigning Joystick: " + object.getPort());
          Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
          Two = new JoystickButton(object, 2);
          Three = new JoystickButton(object, 3);
          Four = new JoystickButton(object, 4);
          Five = new JoystickButton(object, 5);
          Six = new JoystickButton(object, 6);
          Seven = new JoystickButton(object, 7);
          Eight = new JoystickButton(object, 8);
          Nine = new JoystickButton(object, 9);
          Ten = new JoystickButton(object, 10);
          Eleven = new JoystickButton(object, 11);
        }
      }
    }

    public static class PortOne {
      public static class Xbox {
        public static XboxController object = new XboxController(1);
        public static JoystickButton A;
        public static JoystickButton B;
        public static JoystickButton X;
        public static JoystickButton Y;
        public static JoystickButton LeftBumper;
        public static JoystickButton RightBumper;
        public static JoystickButton LeftStickPress;
        public static JoystickButton RightStickPress;
        public static JoystickButton Share;
        public static JoystickButton Options;
        static BooleanSupplier bSupplier = () -> false;
        public static Button LeftTrigger = new Button(bSupplier);
        public static Button RightTrigger = new Button(bSupplier);

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          System.out.println("Assigning static Xbox: " + object.getPort());
          A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
          B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
          X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
          Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
          LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
          RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
          LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
          RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
          Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
          Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
        }
      }

      public static class FlightStick {
        public static Joystick object = new Joystick(1);
        public static JoystickButton Trigger;
        public static JoystickButton Two;
        public static JoystickButton Three;
        public static JoystickButton Four;
        public static JoystickButton Five;
        public static JoystickButton Six;
        public static JoystickButton Seven;
        public static JoystickButton Eight;
        public static JoystickButton Nine;
        public static JoystickButton Ten;
        public static JoystickButton Eleven;

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          assignButtons();
        }

        public static void assignButtons() {
          System.out.println("Assigning Joystick: " + object.getPort());
          Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
          Two = new JoystickButton(object, 2);
          Three = new JoystickButton(object, 3);
          Four = new JoystickButton(object, 4);
          Five = new JoystickButton(object, 5);
          Six = new JoystickButton(object, 6);
          Seven = new JoystickButton(object, 7);
          Eight = new JoystickButton(object, 8);
          Nine = new JoystickButton(object, 9);
          Ten = new JoystickButton(object, 10);
          Eleven = new JoystickButton(object, 11);
        }
      }
    }

    public static class PortTwo {
      public static class Xbox {
        public static XboxController object = new XboxController(2);
        public static JoystickButton A;
        public static JoystickButton B;
        public static JoystickButton X;
        public static JoystickButton Y;
        public static JoystickButton LeftBumper;
        public static JoystickButton RightBumper;
        public static JoystickButton LeftStickPress;
        public static JoystickButton RightStickPress;
        public static JoystickButton Share;
        public static JoystickButton Options;
        static BooleanSupplier bSupplier = () -> false;
        public static Button LeftTrigger = new Button(bSupplier);
        public static Button RightTrigger = new Button(bSupplier);

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          System.out.println("Assigning static Xbox: " + object.getPort());
          A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
          B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
          X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
          Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
          LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
          RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
          LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
          RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
          Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
          Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
        }
      }

      public static class FlightStick {
        public static Joystick object = new Joystick(2);
        public static JoystickButton Trigger;
        public static JoystickButton Two;
        public static JoystickButton Three;
        public static JoystickButton Four;
        public static JoystickButton Five;
        public static JoystickButton Six;
        public static JoystickButton Seven;
        public static JoystickButton Eight;
        public static JoystickButton Nine;
        public static JoystickButton Ten;
        public static JoystickButton Eleven;

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          assignButtons();
        }

        public static void assignButtons() {
          System.out.println("Assigning Joystick: " + object.getPort());
          Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
          Two = new JoystickButton(object, 2);
          Three = new JoystickButton(object, 3);
          Four = new JoystickButton(object, 4);
          Five = new JoystickButton(object, 5);
          Six = new JoystickButton(object, 6);
          Seven = new JoystickButton(object, 7);
          Eight = new JoystickButton(object, 8);
          Nine = new JoystickButton(object, 9);
          Ten = new JoystickButton(object, 10);
          Eleven = new JoystickButton(object, 11);
        }
      }
    }

    public static class PortThree {
      public static class Xbox {
        public static XboxController object = new XboxController(3);
        public static JoystickButton A;
        public static JoystickButton B;
        public static JoystickButton X;
        public static JoystickButton Y;
        public static JoystickButton LeftBumper;
        public static JoystickButton RightBumper;
        public static JoystickButton LeftStickPress;
        public static JoystickButton RightStickPress;
        public static JoystickButton Share;
        public static JoystickButton Options;
        static BooleanSupplier bSupplier = () -> false;
        public static Button LeftTrigger = new Button(bSupplier);
        public static Button RightTrigger = new Button(bSupplier);

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          System.out.println("Assigning static Xbox: " + object.getPort());
          A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
          B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
          X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
          Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
          LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
          RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
          LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
          RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
          Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
          Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
        }
      }

      public static class FlightStick {
        public static Joystick object = new Joystick(3);
        public static JoystickButton Trigger;
        public static JoystickButton Two;
        public static JoystickButton Three;
        public static JoystickButton Four;
        public static JoystickButton Five;
        public static JoystickButton Six;
        public static JoystickButton Seven;
        public static JoystickButton Eight;
        public static JoystickButton Nine;
        public static JoystickButton Ten;
        public static JoystickButton Eleven;

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          assignButtons();
        }

        public static void assignButtons() {
          System.out.println("Assigning Joystick: " + object.getPort());
          Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
          Two = new JoystickButton(object, 2);
          Three = new JoystickButton(object, 3);
          Four = new JoystickButton(object, 4);
          Five = new JoystickButton(object, 5);
          Six = new JoystickButton(object, 6);
          Seven = new JoystickButton(object, 7);
          Eight = new JoystickButton(object, 8);
          Nine = new JoystickButton(object, 9);
          Ten = new JoystickButton(object, 10);
          Eleven = new JoystickButton(object, 11);
        }
      }
    }

    public static class PortFour {
      public static class Xbox {
        public static XboxController object = new XboxController(4);
        public static JoystickButton A;
        public static JoystickButton B;
        public static JoystickButton X;
        public static JoystickButton Y;
        public static JoystickButton LeftBumper;
        public static JoystickButton RightBumper;
        public static JoystickButton LeftStickPress;
        public static JoystickButton RightStickPress;
        public static JoystickButton Share;
        public static JoystickButton Options;
        static BooleanSupplier bSupplier = () -> false;
        public static Button LeftTrigger = new Button(bSupplier);
        public static Button RightTrigger = new Button(bSupplier);

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          System.out.println("Assigning static Xbox: " + object.getPort());
          A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
          B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
          X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
          Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
          LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
          RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
          LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
          RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
          Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
          Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
        }
      }

      public static class FlightStick {
        public static Joystick object = new Joystick(4);
        public static JoystickButton Trigger;
        public static JoystickButton Two;
        public static JoystickButton Three;
        public static JoystickButton Four;
        public static JoystickButton Five;
        public static JoystickButton Six;
        public static JoystickButton Seven;
        public static JoystickButton Eight;
        public static JoystickButton Nine;
        public static JoystickButton Ten;
        public static JoystickButton Eleven;

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          assignButtons();
        }

        public static void assignButtons() {
          System.out.println("Assigning Joystick: " + object.getPort());
          Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
          Two = new JoystickButton(object, 2);
          Three = new JoystickButton(object, 3);
          Four = new JoystickButton(object, 4);
          Five = new JoystickButton(object, 5);
          Six = new JoystickButton(object, 6);
          Seven = new JoystickButton(object, 7);
          Eight = new JoystickButton(object, 8);
          Nine = new JoystickButton(object, 9);
          Ten = new JoystickButton(object, 10);
          Eleven = new JoystickButton(object, 11);
        }
      }
    }

    public static class PortFive {
      public static class Xbox {
        public static XboxController object = new XboxController(5);
        public static JoystickButton A;
        public static JoystickButton B;
        public static JoystickButton X;
        public static JoystickButton Y;
        public static JoystickButton LeftBumper;
        public static JoystickButton RightBumper;
        public static JoystickButton LeftStickPress;
        public static JoystickButton RightStickPress;
        public static JoystickButton Share;
        public static JoystickButton Options;
        static BooleanSupplier bSupplier = () -> false;
        public static Button LeftTrigger = new Button(bSupplier);
        public static Button RightTrigger = new Button(bSupplier);

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          System.out.println("Assigning static Xbox: " + object.getPort());
          A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
          B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
          X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
          Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
          LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
          RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
          LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
          RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
          Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
          Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
        }
      }

      public static class FlightStick {
        public static Joystick object = new Joystick(5);
        public static JoystickButton Trigger;
        public static JoystickButton Two;
        public static JoystickButton Three;
        public static JoystickButton Four;
        public static JoystickButton Five;
        public static JoystickButton Six;
        public static JoystickButton Seven;
        public static JoystickButton Eight;
        public static JoystickButton Nine;
        public static JoystickButton Ten;
        public static JoystickButton Eleven;

        public static void updateController() {
          ControllerTracking.updatePortNumbers();
          assignButtons();
        }

        public static void assignButtons() {
          System.out.println("Assigning Joystick: " + object.getPort());
          Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
          Two = new JoystickButton(object, 2);
          Three = new JoystickButton(object, 3);
          Four = new JoystickButton(object, 4);
          Five = new JoystickButton(object, 5);
          Six = new JoystickButton(object, 6);
          Seven = new JoystickButton(object, 7);
          Eight = new JoystickButton(object, 8);
          Nine = new JoystickButton(object, 9);
          Ten = new JoystickButton(object, 10);
          Eleven = new JoystickButton(object, 11);
        }
      }
    }

    public static void updateAllControllers() {
      if (DriverStation.isJoystickConnected(0)) {
        if (DriverStation.getJoystickType(0) != 20) {
          PortZero.Xbox.updateController();
        } else {
          PortZero.FlightStick.updateController();
        }
      }
      if (DriverStation.isJoystickConnected(1)) {
        if (DriverStation.getJoystickType(1) != 20) {
          PortOne.Xbox.updateController();
        } else {
          PortOne.FlightStick.updateController();
        }
      }
      if (DriverStation.isJoystickConnected(2)) {
        if (DriverStation.getJoystickType(2) != 20) {
          PortTwo.Xbox.updateController();
        } else {
          PortTwo.FlightStick.updateController();
        }
      }
      if (DriverStation.isJoystickConnected(3)) {
        if (DriverStation.getJoystickType(3) != 20) {
          PortThree.Xbox.updateController();
        } else {
          PortThree.FlightStick.updateController();
        }
      }
      if (DriverStation.isJoystickConnected(4)) {
        if (DriverStation.getJoystickType(4) != 20) {
          PortFour.Xbox.updateController();
        } else {
          PortFour.FlightStick.updateController();
        }
      }
      if (DriverStation.isJoystickConnected(5)) {
        if (DriverStation.getJoystickType(5) != 20) {
          PortFive.Xbox.updateController();
        } else {
          PortFive.FlightStick.updateController();
        }
      }
    }
  }

  public static class legacyControllers {
    public static final XboxController controller0 = new XboxController(0);
    public static final JoystickButton con0ButtonA = new JoystickButton(controller0, OIConstants.kXboxButtonA);
    public static final JoystickButton con0ButtonB = new JoystickButton(controller0, OIConstants.kXboxButtonB);
    public static final JoystickButton con0ButtonX = new JoystickButton(controller0, OIConstants.kXboxButtonX);
    public static final JoystickButton con0ButtonY = new JoystickButton(controller0, OIConstants.kXboxButtonY);
    public static final JoystickButton con0ButtonBack = new JoystickButton(controller0, OIConstants.kXboxButtonBack);
    public static final JoystickButton con0ButtonStart = new JoystickButton(controller0, OIConstants.kXboxButtonStart);
    public static final JoystickButton con0BumperLeft = new JoystickButton(controller0, OIConstants.kXboxBumperLeft);
    public static final JoystickButton con0StickPressLeft = new JoystickButton(controller0,
        OIConstants.kXboxStickPressLeft);
    public static final JoystickButton con0StickPressRight = new JoystickButton(controller0,
        OIConstants.kXboxStickPressRight);
    public POVButton con0PovUp = new POVButton(controller0, 0);
    public POVButton con0PovRight = new POVButton(controller0, 90);
    public POVButton con0PovDown = new POVButton(controller0, 180);
    public POVButton con0PovLeft = new POVButton(controller0, 270);

    public static final XboxController controller2 = new XboxController(2);
    public static final JoystickButton con2ButtonA = new JoystickButton(controller2, OIConstants.kXboxButtonA);
    public static final JoystickButton con2ButtonB = new JoystickButton(controller2, OIConstants.kXboxButtonB);
    public static final JoystickButton con2ButtonX = new JoystickButton(controller2, OIConstants.kXboxButtonX);
    public static final JoystickButton con2ButtonY = new JoystickButton(controller2, OIConstants.kXboxButtonY);
    public static final JoystickButton con2ButtonBack = new JoystickButton(controller2, OIConstants.kXboxButtonBack);
    public static final JoystickButton con2ButtonStart = new JoystickButton(controller2, OIConstants.kXboxButtonStart);
    public static final JoystickButton con2BumperLeft = new JoystickButton(controller2, OIConstants.kXboxBumperLeft);
    public static final JoystickButton con2StickPressLeft = new JoystickButton(controller2,
        OIConstants.kXboxStickPressLeft);
    public static final JoystickButton con2StickPressRight = new JoystickButton(controller2,
        OIConstants.kXboxStickPressRight);
    public static final JoystickButton con2Pad = new JoystickButton(controller2,
        OIConstants.kPlaystationDuelsBigButton);
    public static final JoystickButton con2Whatever = new JoystickButton(controller2,
        OIConstants.kPlaystationRightTrigger);
    public POVButton con2PovUp = new POVButton(controller2, 0);
    public POVButton con2PovRight = new POVButton(controller2, 90);
    public POVButton con2PovDown = new POVButton(controller2, 180);
    public POVButton con2PovLeft = new POVButton(controller2, 270);

    public static final Joystick controller5 = new Joystick(5);
    public static final JoystickButton con5Trigger = new JoystickButton(controller5, OIConstants.kATK3BigTrigge);
    public static final JoystickButton con5Button2 = new JoystickButton(controller5, OIConstants.kATK3Button2);
    public static final JoystickButton con5Button3 = new JoystickButton(controller5, OIConstants.kATK3Button3);
    public static final JoystickButton con5Button4 = new JoystickButton(controller5, OIConstants.kATK3Button4);
    public static final JoystickButton con5Button5 = new JoystickButton(controller5, OIConstants.kATK3Button5);
    public static final JoystickButton con5Button6 = new JoystickButton(controller5, OIConstants.kATK3Button6);
    public static final JoystickButton con5Button7 = new JoystickButton(controller5, OIConstants.kATK3Button7);
    public static final JoystickButton con5Button8 = new JoystickButton(controller5, OIConstants.kATK3Button8);
    public static final JoystickButton con5Button9 = new JoystickButton(controller5, OIConstants.kATK3Button9);
    public static final JoystickButton con5Button10 = new JoystickButton(controller5, OIConstants.kATK3Button10);
    public static final JoystickButton con5Button11 = new JoystickButton(controller5, OIConstants.kATK3Button11);
    public static final JoystickButton con5Button12 = new JoystickButton(controller5, OIConstants.kATK3Button12);
  }

  public static class dynamicXbox {
    public static XboxController object = new XboxController(5);
    public static JoystickButton A;
    public static JoystickButton B;
    public static JoystickButton X;
    public static JoystickButton Y;
    public static JoystickButton LeftBumper;
    public static JoystickButton RightBumper;
    public static JoystickButton LeftStickPress;
    public static JoystickButton RightStickPress;
    public static JoystickButton Share;
    public static JoystickButton Options;
    public static POVButton POVUp = new POVButton(object, 0);
    public static POVButton POVDown = new POVButton(object, 180);
    public static POVButton POVLeft = new POVButton(object, 270);
    public static POVButton POVRight = new POVButton(object, 90);
    static BooleanSupplier bSupplier = () -> false;
    public static Button LeftTrigger = new Button(bSupplier);
    public static Button RightTrigger = new Button(bSupplier);

    public static void updateController() {
      ControllerTracking.updatePortNumbers();
      System.out.println("Assigning dXbox: " + object.getPort());
      A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
      B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
      X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
      Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
      LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
      RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
      LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
      RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
      Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
      Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
    }
  }

  public static class dynamicPlaystation {
    public static XboxController object = new XboxController(5);
    public static JoystickButton A;
    public static JoystickButton B;
    public static JoystickButton X;
    public static JoystickButton Y;
    public static JoystickButton LeftBumper;
    public static JoystickButton RightBumper;
    public static JoystickButton LeftStickPress;
    public static JoystickButton RightStickPress;
    public static JoystickButton Share;
    public static JoystickButton Options;
    public static JoystickButton LeftTrigger;
    public static JoystickButton RightTrigger;

    public static void updateController() {
      ControllerTracking.updatePortNumbers();
      assignButtons();
    }

    public static void assignButtons() {
      System.out.println("Assigning dPlaystations: " + object.getPort());
      A = new JoystickButton(object, OIConstants.SmartMap(object, "A"));
      B = new JoystickButton(object, OIConstants.SmartMap(object, "B"));
      X = new JoystickButton(object, OIConstants.SmartMap(object, "X"));
      Y = new JoystickButton(object, OIConstants.SmartMap(object, "Y"));
      LeftBumper = new JoystickButton(object, OIConstants.SmartMap(object, "LBump"));
      RightBumper = new JoystickButton(object, OIConstants.SmartMap(object, "RBump"));
      LeftStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "LStick"));
      RightStickPress = new JoystickButton(object, OIConstants.SmartMap(object, "RStick"));
      Share = new JoystickButton(object, OIConstants.SmartMap(object, "DoubleSquare"));
      Options = new JoystickButton(object, OIConstants.SmartMap(object, "Options"));
      LeftTrigger = new JoystickButton(object, OIConstants.SmartMap(object, "LTrigger"));
      RightTrigger = new JoystickButton(object, OIConstants.SmartMap(object, "RTrigger"));
    }
  }

  public static class dynamicJoystick {
    public static Joystick object = new Joystick(5);
    public static JoystickButton Trigger;
    public static JoystickButton Two;
    public static JoystickButton Three;
    public static JoystickButton Four;
    public static JoystickButton Five;
    public static JoystickButton Six;
    public static JoystickButton Seven;
    public static JoystickButton Eight;
    public static JoystickButton Nine;
    public static JoystickButton Ten;
    public static JoystickButton Eleven;

    public static void updateController() {
      ControllerTracking.updatePortNumbers();
      assignButtons();
    }

    public static void assignButtons() {
      System.out.println("Assigning dJoystick: " + object.getPort());
      Trigger = new JoystickButton(object, OIConstants.SmartMap(object, "Trigger"));
      Two = new JoystickButton(object, 2);
      Three = new JoystickButton(object, 3);
      Four = new JoystickButton(object, 4);
      Five = new JoystickButton(object, 5);
      Six = new JoystickButton(object, 6);
      Seven = new JoystickButton(object, 7);
      Eight = new JoystickButton(object, 8);
      Nine = new JoystickButton(object, 9);
      Ten = new JoystickButton(object, 10);
      Eleven = new JoystickButton(object, 11);
    }
  }

  public RobotContainer() {
    m_driveTrain.setDefaultCommand(new GoTele(false, 0.1, 1));
  }

  public static void configureButtonBindings() {
    System.out.println("Assigning Buttons");

    dynamicXbox.A.whileHeld(() -> System.out.println(dynamicXbox.object.getPort() + ": dynamic XBOX"));
    dynamicJoystick.Three.whileHeld(() -> System.out.println(dynamicJoystick.object.getPort() + ": dynamic JOYSTICK"));
    dynamicPlaystation.A.whileHeld(() -> System.out.println(dynamicPlaystation.object.getPort() + ": dynamic Playstation"));

    dynamicXbox.LeftBumper.whenPressed(() -> CommandScheduler.getInstance().schedule(new AutoSpinToAngleTarget(0.5)));
    dynamicPlaystation.LeftBumper.whenPressed(() -> CommandScheduler.getInstance().schedule(new AutoSpinToAngleTarget(0.5)));

    dynamicXbox.RightBumper.whenPressed(() -> CommandScheduler.getInstance().schedule(new AutoBalance(0.1)));

    dynamicJoystick.Trigger.whenPressed(() -> VisionLight.toggle());
    dynamicXbox.RightTrigger.whenPressed(() -> VisionLight.toggle());
    dynamicPlaystation.RightTrigger.whenPressed(() -> VisionLight.toggle());
    dynamicXbox.B.whileHeld(() -> System.out.println(DriveTrain.m_Gyro.getPitch()));

    dynamicXbox.X.whenPressed(() -> Piston.contract(true));
    dynamicXbox.Y.whenPressed(() -> Piston.contract(false));
    /*
    dynamicXbox.X.whenPressed(() -> TestMotor.SetSpeed(1));
    dynamicXbox.Y.whenPressed(() -> TestMotor.SetSpeed(0));
    dynamicXbox.B.whenPressed(() -> TestMotor.SetSpeed(-1));
    */

    // Possible joystick configuration
    // 4/5 = grab and release | trigger = scoring position | 2 = bottom position | 3 = top/driving position

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public static void RemapControllers() {
    System.out.println("***Started mapping controllers***");
    dynamicXbox.updateController();
    dynamicPlaystation.updateController();
    dynamicJoystick.updateController();
    PortBoundControllers.updateAllControllers();
    System.out.println("***Finished mapping controllers***");
  }
}