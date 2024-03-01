// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.ExampleSubsystem;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;

/** An example command that uses an example subsystem. */
public class ShootingIntakeCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShootingSubsystem shootingSubsystem;
    private final boolean blnShootSpeaker = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public ShootingIntakeCmd(ShootingSubsystem shootingSubsystem, boolean pblnShootSpeaker, boolean blnShootAmp, boolean blnIntakeStar, boolean blnIntakeWheel) {
      //this.open = open;
      this.shootingSubsystem = shootingSubsystem;
      addRequirements(shootingSubsystem);
}
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShootingIntakeCmd Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("ShootingIntakeCmd Executed");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   System.out.println("ShootingIntakeCmd Ended");
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     System.out.println("ShootingIntakeCmd Finished");
    return false;
  }
}
