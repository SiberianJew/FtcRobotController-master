/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

@Config
@TeleOp(name="UrMum2_Redo", group="Linear OpMode")
//@Disabled
public class UrMum2_Redo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private PIDController controller;
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private DcMotor biggerSucc = null;
    //Inner intake
    private DcMotor longSticL = null;
    //Right linear slide
    private DcMotor longSticR = null;
    //Left linear slide
    private DcMotor turnOnner = null;
    //Slide rotater
    private Servo bigDumpie = null;
    //Hex dumper
    private Servo bigDumpieTwo = null;

    //private Servo bigDumpie,bigDumpieTwo;

    //Hex dumper 2
    //private final double ticks_in_degree = 84 / 360;
    //public static double p = 0,i = 0,d = 0;
    //public static double f = 0;
    //public static int target = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        //controller = new PIDController(p, i, d);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        turnOnner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnOnner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnOnner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        biggerSucc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        longSticL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        longSticR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double x = 0.1;
        bigDumpie.setDirection(Servo.Direction.FORWARD);
        bigDumpieTwo.setDirection(Servo.Direction.REVERSE);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double speed = 1;

            motorFR.setPower((gamepad1.left_stick_y * speed) + gamepad1.right_trigger - gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorBR.setPower((gamepad1.left_stick_y * speed) - gamepad1.right_trigger + gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorFL.setPower((gamepad1.left_stick_y * speed) - gamepad1.right_trigger + gamepad1.left_trigger - (gamepad1.right_stick_x * speed));
            motorBL.setPower((gamepad1.left_stick_y * speed) + gamepad1.right_trigger - gamepad1.left_trigger - (gamepad1.right_stick_x * speed));

            if (gamepad2.right_trigger > .1) {
                biggerSucc.setPower(1.0);
            } else if (gamepad2.left_trigger > .1) {
                biggerSucc.setPower(-.5);
            } else {
                biggerSucc.setPower(0);
            }

            

            if (gamepad2.left_stick_y > 0) {
                longSticL.setPower(-1);
                longSticR.setPower(-1);
            } else if (gamepad2.left_stick_y < 0) {
                longSticL.setPower(1);
                longSticR.setPower(1);
            } else {
                longSticL.setPower(0);
                longSticR.setPower(0);
            }

            /*longSticL.setPower(-gamepad2.left_stick_y);
              longSticR.setPower(-gamepad2.right_stick_y);
             */

            /* if (gamepad2.x) {
                turnOnner.setPower(-1);
            } else if (gamepad2.y) {
                turnOnner.setPower(.5);
            } else {
                turnOnner.setPower(0.001);
            } */

            /* controller.setPID(p, i, d);
            int armPos = turnOnner.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toDegrees(target / ticks_in_degree)) * f;

            double power = pid + ff;

            if (gamepad2.x) {
                turnOnner.setPower(power);
            } else {
                turnOnner.setPower(0);
            } */

            if (gamepad2.a) {
                bigDumpie.setPosition(x += 0.08);
            } else  {
                bigDumpie.setPosition(x = 0.1);
            }
            //bigDumpie.setPosition(x=gamepad2.a?x+0.08:0.1);
            bigDumpieTwo.setPosition(bigDumpie.getPosition());

            // Show the elapsed game time and wheel power.
            // telemetry.addData("pos", armPos);
            //telemetry.addData("target", target);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
