package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootingSubsystem extends SubsystemBase {

    public static final ShootingSubsystem instance = new ShootingSubsystem();

    public final Spark intakeMotor = new Spark(Constants.OperatorConstants.kIntakePWM);
    public final Spark shooterMotor = new Spark(Constants.OperatorConstants.kShooterPWM);
    public final Spark primerMotor1 = new Spark(Constants.OperatorConstants.kPrimer1PWM);
    public final Spark primerMotor2 = new Spark(Constants.OperatorConstants.kPrimer2PWM);
    public boolean isPrimed = false;

    public void shootingSubsystem() {}

   /**
   * Turns on the priming motors.
   * @param shootingMode either 0 for low power or 1 for high power.
   */
  public Command togglePriming(int shootingMode) {
    return runOnce(
        () -> {

            if (!isPrimed) {

                isPrimed = true;

                if (shootingMode == 1) {
                    primerMotor1.set(1);
                    primerMotor2.set(1);
                } else if (shootingMode == 0) {
                    primerMotor1.set(.5);
                    primerMotor2.set(.5);
                }
            } else if (isPrimed) {

                isPrimed = false;

                primerMotor1.stopMotor();
                primerMotor2.stopMotor();
            }

        });
    }

      /*
       * Turns on the shooter motor.
       */
      public Command shoot() {
        return runOnce(
            () -> {
              shooterMotor.set(1);
            });
      }

      /*
       * Turns off the shooter motor.
       */
      public Command unShoot() {
        return runOnce(
            () -> {
              shooterMotor.stopMotor();
            });
      }

    public static ShootingSubsystem getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
    }
    
}
