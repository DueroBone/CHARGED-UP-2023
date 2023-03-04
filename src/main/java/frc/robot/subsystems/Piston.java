package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Piston {
  private static DoubleSolenoid piston;

  public Piston() {
    piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public static Command contract(final boolean contracted) {
    System.out.println("Piston extended: " + contracted);
    // piston0.set(contracted);
    // piston1.set(!contracted);

    if (contracted) { // if true then shift to low gear
      piston.set(DoubleSolenoid.Value.kForward); // solenoid controls output that pulls piston in or out
    } else {
      piston.set(DoubleSolenoid.Value.kReverse);
    }
    return null;
  }

  public static void pistonToggle() {
    piston.toggle();
  }
}
