package frc.robot.movement;

public class Movement {

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
