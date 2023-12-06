package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class UrMum2 extends OpMode {

    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;
    DcMotor bigSucc = null;
    DcMotor biggerSucc = null;
    DcMotor longStic = null;
    DcMotor turnOnner = null;
    Servo bigDumpie = null;
    //Servo gyatToTurn = null;


    public void driveTrainDriving() {

        double vertical;
        double horizontal;

        vertical = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;

        motorFR.setPower(vertical +  horizontal);
        motorBR.setPower(vertical - horizontal);
        motorFL.setPower(vertical - horizontal);
        motorBL.setPower(-vertical - horizontal);

    }

    public void driveTrainRotating() {

        double rotate;

        rotate = gamepad1.right_stick_x;

        motorFR.setPower(rotate);
        motorBR.setPower(rotate);
        motorFL.setPower(-rotate);
        motorBL.setPower(rotate);
    }

    public void triggers() {

        double tl = gamepad1.left_trigger;
        double tr = gamepad1.right_trigger;

        double succIn = gamepad2.right_trigger;
        double unSucc = gamepad2.left_trigger;

        motorFR.setPower(tr - tl);
        motorBR.setPower(-tr + tl);
        motorFL.setPower(-tr + tl);
        motorBL.setPower(-tr + tl);

        bigSucc.setPower(succIn - unSucc);
        bigSucc.setPower(-unSucc);

    }

    public void sticks() {

        double stic = gamepad2.left_stick_y;
        double suc = gamepad2.right_stick_y;

        /*if (longStic.getCurrentPosition() > 0 || longStic.getCurrentPosition() > 0) {
            longStic.setPower(stic);
        }
        else {
            longStic.setPower(0);
        } */
        longStic.setPower(stic);
        biggerSucc.setPower(suc);

    }

    public void letters() {
        /*
        if (gamepad2.x) {
            // When the 'X' button on Gamepad2 is pressed, move the motor clockwise.
            turnOnner.setPower(0.5); // Adjust the power level as needed.
        } else if (gamepad2.y) {
            // When the 'Y' button on Gamepad2 is pressed, move the motor counter-clockwise.
            turnOnner.setPower(-0.5); // Adjust the power level as needed.
        }   else if (gamepad2.a) {
                turnOnner.setPower(.2);
            }
        else {
            turnOnner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turnOnner.setPower(0);
        }
         */



        if (gamepad2.x) {

            int degrees0 = 300;

            turnOnner.setTargetPosition(degrees0);
            turnOnner.setPower(.5);
            turnOnner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.y) {

            int degrees0 = 500;

            turnOnner.setTargetPosition(degrees0);
            turnOnner.setPower(.5);
            turnOnner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.a) {
            int degrees0 = 100;

            turnOnner.setTargetPosition(degrees0);
            turnOnner.setPower(.5);
            turnOnner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
    }

    public void dPads() {
        if (gamepad2.dpad_up) {
            // When the 'X' button on Gamepad2 is pressed, move the motor clockwise.
            bigDumpie.setPosition(0.85); // Adjust the power level as needed.
        } else {
            bigDumpie.setPosition(0.4);
        }
    }

    @Override
    public void init() {
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        bigSucc = hardwareMap.get(DcMotor.class, "bigSucc");
        biggerSucc = hardwareMap.get(DcMotor.class, "biggerSucc");
        longStic = hardwareMap.get(DcMotor.class, "longStic");
        turnOnner = hardwareMap.get(DcMotor.class, "turnOnner");
        bigDumpie = hardwareMap.get(Servo.class, "bigDumpie");

        turnOnner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnOnner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        longStic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        longStic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Hardware: ", "Initialized");



    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

        bigSucc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        biggerSucc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        longStic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrainDriving();
        driveTrainRotating();
        triggers();
        sticks();
        letters();
        dPads();
    }

    /* float xL = (float) (gamepad1.left_stick_x*0.5);
    float yL = (float) (gamepad2.left_stick_y*0.5);

    EXEPTIONS TO DIAGONALS IF VARIABLES ARE WITHIN A SIMILAR VALUE...

        if (true) {
        motorBL.setPower(yL);
        motorBR.setPower(-yL);
    } */

}
