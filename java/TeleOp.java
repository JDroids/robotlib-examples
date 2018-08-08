package org.firstinspires.ftc.teamcode.java;


import com.jdroids.robotlib.gamepad.EnhancedGamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.java.robot.Robot;
import org.firstinspires.ftc.teamcode.java.robot.subsystems.Drive;

/**
 * This is a simple tank drive Teleop class written in Kotlin.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SimpleTeleopJava")
public class TeleOp extends OpMode {
    private final EnhancedGamepad enhancedGamepad1 = new EnhancedGamepad(gamepad1);

    @Override
    public void init() {
        Robot.getInstance().teleopInit();
    }

    @Override
    public void loop() {
        double deadband = 0.02;

        final float leftPower = -enhancedGamepad1.getJoystick(EnhancedGamepad.Hand.LEFT,
                EnhancedGamepad.Direction.Y, deadband);
        final float rightPower = -enhancedGamepad1.getJoystick(EnhancedGamepad.Hand.RIGHT,
                EnhancedGamepad.Direction.Y, deadband);

        Robot.getInstance().drive.driveOpenLoop(new Drive.DriveSignal(leftPower, rightPower));
    }
}
