package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Base side + Crater")
public class Autonomous_Base_Robot extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.Holder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Holder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Holder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Joint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        ;
      //  encoderMotor(DRIVE_SPEED, 1.5, 3.0);
      //  encoderDrive(TURN_SPEED, 5, -5, 3.0);

        encoderDrive(DRIVE_SPEED, -35, -35, 1.5);
        encoderDrive(DRIVE_SPEED, 2, 2, .2);
        encoderDrive(DRIVE_SPEED, -2, -2, .2);
        encoderDrive(DRIVE_SPEED, 2, 2, .2);
        encoderDrive(TURN_SPEED, -4.4, 4.4, .248);
        encoderDrive(DRIVE_SPEED, 59, 59, 3.0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newLeftTarget1;
        int newRightTarget;
        int newRightTarget1;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget1 = robot.FLMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftTarget = robot.BLMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget1 = robot.FRMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTarget = robot.BLMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.FLMotor.setTargetPosition(newLeftTarget1);
            robot.BLMotor.setTargetPosition(newLeftTarget);
            robot.FRMotor.setTargetPosition(newRightTarget1);
            robot.BRMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FRMotor.setPower(Math.abs(speed));
            robot.BRMotor.setPower(Math.abs(speed));
            robot.FLMotor.setPower(Math.abs(speed));
            robot.BLMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FLMotor.isBusy() && robot.FRMotor.isBusy()) &&
                    (robot.BRMotor.isBusy() && robot.BRMotor.isBusy())) {
            }

            // Stop all motion;
            robot.FLMotor.setPower(0);
            robot.BLMotor.setPower(0);
            robot.FRMotor.setPower(0);
            robot.BRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }



  /*  public void encoderMotor(double speed,
                             double inches,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.hook.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.hook.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.hook.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.hook.isBusy())) {
            }

            // Stop all motion;
            robot.hook.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(100);   // optional pause after each move
        }

    }
*/

}