package org.firstinspires.ftc.teamcode.kotlin.robot.subsystems

import android.util.Log
import com.jdroids.robotlib.command.Subsystem
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.kotlin.robot.commands.StayInPlace

/**
 * This is a kotlin example demonstrating robotlib [subsystems][Subsystem]. This represents the
 * drivetrain of the v4a pushbot which can be found [here](https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/pushbot-build-guide-vertical.pdf).
 */
class Drive : Subsystem() {
    private val left
            by lazy{hardwareMap!!.get(DcMotorEx::class.java, "left")}
    private val right
            by lazy {hardwareMap!!.get(DcMotorEx::class.java, "right")}

    private val hub
            by lazy {hardwareMap!!.get(LynxModule::class.java, "hub")}

    private var leftPower = 0.0
    private var rightPower = 0.0

    var leftPosition = 0
        private set
    var rightPosition = 0
        private set

    override fun initHardware(hardwareMap: HardwareMap) {
        left.mode = DcMotor.RunMode.RUN_USING_ENCODER
        right.mode = DcMotor.RunMode.RUN_USING_ENCODER

        super.initHardware(hardwareMap)
    }

    override fun initDefaultCommand() {}

    override fun periodic() {
        left.power = leftPower
        right.power = rightPower

        getEncoderPositions()
    }

    fun driveOpenLoop(signal: DriveSignal) {
        leftPower = signal.leftPower.toDouble()
        rightPower = signal.rightPower.toDouble()
    }

    data class DriveSignal(var leftPower: Number, var rightPower: Number)

    private fun getEncoderPositions() {
        val command = LynxGetBulkInputDataCommand(hub)

        try {
            val response = command.sendReceive()

            //The left motor has to be plugged into the 0 port for this to work
            leftPosition = response.getEncoder(0)
            //The right motor has to be plugged into the 1 port for this to work
            rightPosition = response.getEncoder(1)
        }
        catch (e : Exception) {
            Log.w(this.javaClass.name, e)
        }
    }
}