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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TOGearShift")
public class PushbotTeleopPOV_Linear extends LinearOpMode {


    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        //up and down joints are for the motor closest to the base of the robot while
        // the up and down Holder is the motor closest to the attachment actually holding the balls
     //   double upJoint;
     //   double downJoint;
      //  double upHolder;
        //  double downHolder;
        double Right;
        double Left;
        //double upArm;
       // double downArm;
        double upHook;
        double downHook;

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

         // upJoint = gamepad2.left_stick_y * 10;
         // downJoint = -gamepad2.left_stick_y * 10;
            Right = -gamepad1.right_stick_y * (Math.abs(-gamepad1.right_stick_y)/10);
            Left = -gamepad1.left_stick_y * (Math.abs(-gamepad1.right_stick_y)/10);
          //  upHolder = gamepad2.right_stick_y * 10;
           // downHolder = -gamepad2.right_stick_y * 10;
           // upArm = gamepad2. left_trigger * (Math.abs (gamepad2.left_trigger)/10);
           // downArm = gamepad2.right_trigger * (Math.abs (gamepad2.right_trigger)/10);
            upHook = gamepad1.left_trigger;
            downHook = gamepad1.right_trigger;


            robot.FLMotor.setPower(Left);
            robot.FRMotor.setPower(Right);
            robot.BLMotor.setPower(Left);
            robot.BRMotor.setPower(Right);
        //    robot.Holder.setPower (upHolder);
        //    robot.Holder.setPower (downHolder);
          // robot.Joint.setPower (upJoint);
         //  robot.Joint.setPower (downJoint);
            robot.hook.setPower (upHook);
            robot.hook.setPower (downHook);
           // robot.arm.setPosition(upArm);
           // robot.arm.setPosition(downArm);

        }
    }


}
