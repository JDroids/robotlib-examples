package org.firstinspires.ftc.teamcode.kotlin

import com.jdroids.robotlib.gamepad.EnhancedGamepad
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.kotlin.robot.Robot
import org.firstinspires.ftc.teamcode.kotlin.robot.subsystems.Drive

/**
 * This is a simple tank drive Teleop class written in Kotlin.
 */
@Disabled
@TeleOp(name="SimpleTeleOpKotlin")
class TeleOp : OpMode() {
    private val enhancedGamepad1 = EnhancedGamepad(gamepad1)

    override fun init() {
        Robot.teleopInit()
    }

    override fun loop() {
        val leftPower = -enhancedGamepad1.getJoystick(EnhancedGamepad.Hand.LEFT,
                EnhancedGamepad.Direction.Y)
        val rightPower = -enhancedGamepad1.getJoystick(EnhancedGamepad.Hand.RIGHT,
                EnhancedGamepad.Direction.Y)

        Robot.drive.driveOpenLoop(Drive.DriveSignal(leftPower, rightPower))
    }
}