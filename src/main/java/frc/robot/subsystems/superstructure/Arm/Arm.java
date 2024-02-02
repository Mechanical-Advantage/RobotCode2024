package frc.robot.subsystems.superstructure.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
   private final ArmIO io;
   //prob change this maybe i think
   private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();
   private final LoggedTunableNumber armP = new LoggedTunableNumber("Arm/P", 6.0);
   private final LoggedTunableNumber armI = new LoggedTunableNumber("Arm/I", 3.0);
   private final LoggedTunableNumber armD = new LoggedTunableNumber("Arm/D", 2.0);
   private final LoggedTunableNumber armF = new LoggedTunableNumber("Arm/F", 8.0);
   private final LoggedTunableNumber armMaxVelocity = new LoggedTunableNumber("Arm/MaxVelocity", 6.3);
   private final LoggedTunableNumber armMaxAcceleration = new LoggedTunableNumber("Arm/MaxAcceleration", 2.8);

   public Arm(ArmIO io) {
      System.out.println("[Init] Creating Arm");
      this.io = io;


   }

}
