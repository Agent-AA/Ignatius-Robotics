package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShootingSubsystem extends SubsystemBase {
    // driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    //private final CANSparkMax driveMotor;
    // Spark intakeMotor;
    // Spark shooterMotor;
    // Spark primerMotor1;
    // Spark primerMotor2;

    private boolean prevChangeShootSpeakerButtonValue = false;
    private boolean prevChangeShootAmpButtonValue = false;
    private boolean prevChangeIntakeCtrlButtonValue = false;
    private boolean prevChangeShooterCtrlButtonValue = false;

    private Spark intakeMotor = new Spark(Constants.OperatorConstants.kIntakePWM);
    private Spark shooterMotor = new Spark(Constants.OperatorConstants.kShooterPWM);
    private Spark primerMotor1 = new Spark(Constants.OperatorConstants.kPrimer1PWM);
    private Spark primerMotor2 = new Spark(Constants.OperatorConstants.kPrimer2PWM);

    public boolean shootSpeaker;
    public boolean shootAmp;
    public boolean intakeMotorCtrl;
    public boolean shooterMotorCtrl;

    //intakeMotor = new Spark(Constants.OperatorConstants.kIntakePWM);    //0
    //shooterMotor = new Spark(Constants.OperatorConstants.kShooterPWM);  //1
    //primerMotor1 = new Spark(Constants.OperatorConstants.kPrimer1PWM);  //2
    //primerMotor2 = new Spark(Constants.OperatorConstants.kPrimer2PWM);  //3
    public boolean isPrimed = false;

    //public static final ShootingSubsystem instance = new ShootingSubsystem();
    
    public void shootingSubsystem() {
        //intakeMotor = new Spark(Constants.OperatorConstants.kIntakePWM);
        //shooterMotor = new Spark(Constants.OperatorConstants.kShooterPWM);  //1
        //primerMotor1 = new Spark(Constants.OperatorConstants.kPrimer1PWM);  //2
        //primerMotor2 = new Spark(Constants.OperatorConstants.kPrimer2PWM); 
        //  () -> {
         
        // primerMotor1.set(1);
        // primerMotor2.set(1);
        //  }
        //primerMotor1.set(0);
        //primerMotor2.set(0);
    }
    
    // public static ShootingSubsystem getInstance() {
    //     return instance;
    // }
    @Override
    public void periodic() {
        // Called once per scheduler run
        //primerMotor1.set(0); //SHOULD GET FROM CONSTANTS EVENTUALLY
        //primerMotor2.set(0);
    }
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
    }
    public void mtdShootSpeaker(Boolean changeShootSpeakerButtonValue, Boolean changeShootAmpButtonValue) { //, Spark primeMotor1, Spark primeMotor2) {
    //  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   
        SmartDashboard.putBoolean("SHOOT SPEAKER ON Test!()", shootSpeaker);
        SmartDashboard.putBoolean("SHOOT AMP ON Test!()", shootAmp);
        if (!prevChangeShootSpeakerButtonValue && changeShootSpeakerButtonValue) {
            shootSpeaker = !shootSpeaker;
            shootAmp = false;
        }
        else if (!prevChangeShootAmpButtonValue && changeShootAmpButtonValue) {
             shootAmp = !shootAmp;
             shootSpeaker = false; //shootAmp; //added to fix button issue
          }
        prevChangeShootSpeakerButtonValue = changeShootSpeakerButtonValue;
        prevChangeShootAmpButtonValue = changeShootAmpButtonValue;

        if (shootSpeaker){
            System.out.println("SHOOT SPEAKER BUTTON HAS BEEN ENGAGED TO START SHOOTER!!!!");
            System.out.println("SHOOT AMP BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
            primerMotor1.set(Constants.OperatorConstants.kPrimerSpeaker); 
            primerMotor2.set(Constants.OperatorConstants.kPrimerSpeaker);
        }
        else if (shootAmp) {
            System.out.println("SHOOT SPEAKER BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
            System.out.println("SHOOT AMP BUTTON HAS BEEN ENGAGED TO START SHOOTER!!!!");
            primerMotor1.set(Constants.OperatorConstants.kPrimerAmp); 
            primerMotor2.set(Constants.OperatorConstants.kPrimerAmp);
        }
        else
        {
            System.out.println("SHOOT SPEAKER BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
            System.out.println("SHOOT AMP BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
            primerMotor1.set(0);
            primerMotor2.set(0);
        }
    }

    // Shoot Amp combined with shootSpeaker to address command conflict.

    // public void mtdShootAmp(Boolean changeShootAmpButtonValue) { //, Spark primeMotor1, Spark primeMotor2) {
    // // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   
       
    //     SmartDashboard.putBoolean("SHOOT AMP ON Test!()", shootAmp);
    //     if (!prevChangeShootAmpButtonValue && changeShootAmpButtonValue) {
    //         shootAmp = !shootAmp;
    //         shootSpeaker = shootAmp; //added to fix button issue
    //     }
    //     prevChangeShootAmpButtonValue = changeShootAmpButtonValue;

    //     if (shootAmp == true){
    //         System.out.println("SHOOT AMP BUTTON HAS BEEN ENGAGED TO START SHOOTER!!!!");
    //         primerMotor1.set(Constants.OperatorConstants.kPrimerAmp); 
    //         primerMotor2.set(Constants.OperatorConstants.kPrimerAmp);
    //     }
    //     else
    //     {
    //         System.out.println("SHOOT AMP BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
    //         primerMotor1.set(0);
    //         primerMotor2.set(0);
    //     }
    //}


    public void mtdIntake(Boolean changeIntakeCtrlButtonValue) { //, Spark primeMotor1, Spark primeMotor2) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   
        SmartDashboard.putBoolean("INTAKE STAR ROLLERS ON Test!()", intakeMotorCtrl);
        if (!prevChangeIntakeCtrlButtonValue && changeIntakeCtrlButtonValue) {
            intakeMotorCtrl = !intakeMotorCtrl;
        }
        prevChangeIntakeCtrlButtonValue = changeIntakeCtrlButtonValue;

        if (intakeMotorCtrl == true){
            System.out.println("INTAKE BUTTON HAS BEEN ENGAGED TO START SHOOTER!!!!");
            intakeMotor.set(Constants.OperatorConstants.kIntakeMotor); 
        }
        else
        {
            System.out.println("INTAKE BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
            intakeMotor.set(0);
        }
    }
    public void mtdShooter(Boolean changeShooterCtrlButtonValue) { //, Spark primeMotor1, Spark primeMotor2) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   
        SmartDashboard.putBoolean("INTAKE WHEEL ROLLERSON Test!()", shooterMotorCtrl);

        if (!prevChangeShooterCtrlButtonValue && changeShooterCtrlButtonValue) {
            shooterMotorCtrl = !shooterMotorCtrl;
        }
        prevChangeShooterCtrlButtonValue = changeShooterCtrlButtonValue;

        if (shooterMotorCtrl == true){
            System.out.println("SHOOTER BUTTON HAS BEEN ENGAGED TO START SHOOTER!!!!");
            shooterMotor.set(Constants.OperatorConstants.kShooterMotor); 
        }
        else
        {
            System.out.println("SHOOTER BUTTON HAS BEEN DISENGAGED TO STOP SHOOTER!!!!");
            shooterMotor.set(0);
        }
    }
}
