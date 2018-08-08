package org.firstinspires.ftc.teamcode.kotlin

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.kotlin.robot.Robot
import org.firstinspires.ftc.teamcode.kotlin.robot.commands.StayInPlace

/**
 * This represents a simple kotlin autonomous that tries to stay in the same spot for the whole
 * autonomous period.
 */
@Disabled
@Autonomous(name="SimpleAutoKotlin")
class Autonomous : OpMode() {
    private val stayInPlace = StayInPlace()

    override fun init() {
        Robot.autonomousInit()
    }

    override fun start() {
        stayInPlace.start()
    }

    override fun loop() {}

    override fun stop() {
        stayInPlace.cancel()
    }
}