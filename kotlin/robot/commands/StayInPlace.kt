package org.firstinspires.ftc.teamcode.kotlin.robot.commands

import com.jdroids.robotlib.command.Command
import com.jdroids.robotlib.pid.PIDOutput
import com.jdroids.robotlib.pid.PIDSource
import com.jdroids.robotlib.pid.SimplePIDController
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.teamcode.kotlin.robot.Robot
import org.firstinspires.ftc.teamcode.kotlin.robot.subsystems.Drive

/**
 * This is a simple Command demonstrating how to use a Command in Kotlin, along with how to use
 * a SimplePIDController.
 */
class StayInPlace : Command() {
    init {
        requires(Robot.drive)
    }

    private val drive = Robot.drive
    private val signal = Drive.DriveSignal(0.0, 0.0)

    private val sources = arrayOf(
            MotorPositionPIDSource { drive.leftPosition },
            MotorPositionPIDSource { drive.rightPosition })
    private val outputs = arrayOf(
            MotorPowerPIDOutput { power -> signal.leftPower = power },
            MotorPowerPIDOutput { power -> signal.rightPower = power }
    )
    private val coeffients = arrayOf(
            //These numbers are samples and not representative of any actual PID coefficients
            PIDCoefficients(0.43, 0.2, 0.6), PIDCoefficients(0.24, 0.4, 0.3)
    )

    private val controllers =
            Array(2) {i -> SimplePIDController(sources[i], outputs[i], coeffients[i])}
    private val targetPositions = Array(2) {0}


    private class MotorPositionPIDSource(private val getMotorPosition: () -> Int) : PIDSource {
        override fun getPIDSourceType(): PIDSource.Type = PIDSource.Type.DISPLACEMENT

        override fun setPIDSourceType(type: PIDSource.Type) {}

        override fun pidGet(): Double = getMotorPosition().toDouble()
    }

    private class MotorPowerPIDOutput(private val setPower: (power: Double) -> Unit) : PIDOutput {
        override fun pidWrite(output: Double) = setPower(output)
    }

    override fun isFinished(): Boolean = false

    override fun initialize() {
        targetPositions[0] = drive.leftPosition
        targetPositions[1] = drive.rightPosition

        for ((i, controller) in controllers.withIndex()) {
            controller.setSetpoint(targetPositions[i].toDouble())
            controller.enable()
        }

        super.initialize()
    }

    override fun execute() {
        controllers.forEach{it.calculate()}

        drive.driveOpenLoop(signal)
    }
}