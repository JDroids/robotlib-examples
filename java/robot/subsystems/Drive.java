package org.firstinspires.ftc.teamcode.java.robot.subsystems;

import android.util.Log;

import com.jdroids.robotlib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is a Java example demonstrating robotlib [subsystems][Subsystem]. This represents the
 * drivetrain of the v4a pushbot which can be found [here](https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/pushbot-build-guide-vertical.pdf).
 */
public class Drive extends Subsystem {
    private DcMotorEx left;
    private DcMotorEx right;

    private LynxModule hub;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    private int leftPosition = 0;
    private int rightPosition = 0;

    @Override
    public void initHardware(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        hub = hardwareMap.get(LynxModule.class, "hub");

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void initDefaultCommand() {}

    @Override
    public void periodic() {
        left.setPower(leftPower);
        right.setPower(rightPower);

        getEncoderPositions();
    }

    public void driveOpenLoop(DriveSignal signal) {
        leftPower = signal.leftPower;
        rightPower = signal.rightPower;
    }

    static public class DriveSignal {
        public double leftPower;
        public double rightPower;

        public DriveSignal(double leftPower, double rightPower) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
        }
    }

    public enum Motor {
        LEFT,
        RIGHT
    }

    public int getPosition(Motor motor) {
        switch(motor) {
            case LEFT:
                return getLeftPosition();
            case RIGHT:
                return getRightPosition();
        }
        return Integer.MIN_VALUE; //This should never occur, as the above switch is exhaustive
    }

    public int getLeftPosition() {
        return leftPosition;
    }

    public int getRightPosition() {
        return rightPosition;
    }

    private void getEncoderPositions() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(hub);

        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();

            //The left motor has to be plugged into the 0 port for this to work
            leftPosition = response.getEncoder(0);
            //The right motor has to be plugged into the 1 port for this to work
            rightPosition = response.getEncoder(1);
        }
        catch (Exception e) {
            Log.w(this.getClass().getName(), e);
        }
    }
}
