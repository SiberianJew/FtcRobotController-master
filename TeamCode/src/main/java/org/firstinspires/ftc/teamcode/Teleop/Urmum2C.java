package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
@ // /* */

/**Extension of LinearOpMode customed for Carlos
 * extends - LinearOpMode
 */
public class Urmum2C extends LinearOpMode{
    protected DcMotor motorFR,motorFL,motorBR,motorBL,biggerSucc,longSticL,longSticR,turnOnner;
    private Servo bigDumpie,bigDumpieTwo;
    @Override
    /** runner
     * @input n/a
     * @output void
     */
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        biggerSucc = hardwareMap.get(DcMotor.class, "biggerSucc");
        longSticL = hardwareMap.get(DcMotor.class, "longSticL");
        longSticR = hardwareMap.get(DcMotor.class, "longSticR");
        turnOnner = hardwareMap.get(DcMotor.class, "turnOnner");
        bigDumpie = hardwareMap.servo.get("bigDumpie");
        bigDumpieTwo = hardwareMap.servo.get("bigDumpieTwo");

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        turnOnner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnOnner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnOnner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        biggerSucc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        longSticL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        longSticR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        double x = 0.1,speed=1;
        bigDumpie.setDirection(Servo.Direction.FORWARD);
        bigDumpieTwo.setDirection(Servo.Direction.REVERSE);

        while (opModeIsActive()) {
            motorFR.setPower((gamepad1.left_stick_y + gamepad1.right_trigger - gamepad1.left_trigger + gamepad1.right_stick_x) * speed);
            motorBR.setPower((gamepad1.left_stick_y - gamepad1.right_trigger + gamepad1.left_trigger + gamepad1.right_stick_x) * speed);
            motorFL.setPower((gamepad1.left_stick_y - gamepad1.right_trigger + gamepad1.left_trigger - gamepad1.right_stick_x) * speed);
            motorBL.setPower((gamepad1.left_stick_y + gamepad1.right_trigger - gamepad1.left_trigger - gamepad1.right_stick_x) * speed);

            biggerSucc.setPower(gamepad2.right_trigger>.1?1:gamepad2.left_trigger>.1?-.5:0);

            longSticL.setPower(-gamepad2.left_stick_y);
            longSticR.setPower(-gamepad2.left_stick_y);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
/*class x{
    public void y(){
    }
}*/