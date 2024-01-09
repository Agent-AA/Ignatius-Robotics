package frc.robot.movement;

public class Movement {

    /* These objects are the Pulse Width Modulation (PWM) controllers. They are how
     * the RoboRIO controls the motors. The _drive objects control the actual drive;
     * the _swerve objects control the direction of the corresponding wheel.
    */
    private final PWMSparkMax left_front_drive = new PWMSparkMax(0);
    private final PWMSparkMax left_front_swerve = new PWMSparkMax(1);
    private final PWMSparkMax left_back_drive = new PWMSparkMax(2);
    private final PWMSparkMax left_back_swerve = new PWMSparkMax(3);
    private final PWMSparkMax right_front_drive = new PWMSparkMax(4);
    private final PWMSparkMax right_front_swerve = new PWMSparkMax(5);
    private final PWMSparkMax right_back_drive = new PWMSparkMax(6);
    private final PWMSparkMax right_back_swerve = new PWMSparkMax(7);

    // TODO: add unit for moveTime parameter
    /**
     * Moves the robot in a given direction for a given amount of time.
     * When a parameter is null, the method uses the last value given for that parameter.
     * @param moveTime the amount of time to move for
     * @param moveSpeed nullable; the speed to move at, in meters per second
     * @param bodyAngle nullable; the direction that the robot is facing, as an angle in degrees.
     * @param moveAngle nullable; the direction to move in, as an angle in degrees.
     */
    public static void move(float moveTime, float moveSpeed, float bodyAngle, float moveAngle) {}

    /**
     * Sets the current direction (angle) of the robot's bodyAngle
     * to 0 degrees ("north"); essentially creates a new
     * point of reference for the robot.
     */
    public static void setNorth() {}
}
