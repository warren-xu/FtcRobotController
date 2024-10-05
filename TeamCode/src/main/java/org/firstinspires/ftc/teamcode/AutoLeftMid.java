/* Copyright (c) 2021 FIRST. All rights reserved.
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

import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.List;


@Autonomous(name="Left Medium High Pole")
//@Disabled

public class AutoLeftMid extends LinearOpMode {
    private static CameraObjectDetector cameraObjectDetector = null;
    private static ChassisSystem chassis = null;
    private static Odometry odometry = null;
   // MecanumXYTurnDriveLib mecanumDrive = null;
    private static MecanumDriveLib chassisController = null;

    private static HSliderSystem hSliderSystem = null;
    private static VSliderSystem vSliderSystem = null;
    private static RobotPosition robotPos = new RobotPosition(0,0,0);

    private static RobotPosition parkPosition = new RobotPosition(0,0,0);
    // the start position and final target position
    private static final double STARTX = -1.56;
    private static final double STARTY = 0.8 ;
    private static final double STARTA = 0;

    private static double targetX = STARTX +  1.19;
    private static double targetY = STARTY + 0.15;
    private static double targetA = -98;
    private static  double targetVel = 1.0;
    public static double turnVel = 0.7;


    //for pick up
    private double HPICKUP_LEN = 0.31;
    private double HPICKUP_TILT_ANGLE = -103;
    private double HPICKUP_PAN_ANGLE = -1.25;

    // for handover
    private final double HHANDOVER_MIDDLE_TILT_ANGLE = 5.86;

    private final double HHANDOVER_TILT_ANGLE = 36;
    private final double HHANDOVER_PAN_ANGLE = 0;
    private final double HSLIDER_HANDOVER_LEN = 0.025;


    private final double VHANDOVER_LEN = 0.15;

    private final double VHANDOVER_TILT_ANGLE = -126.13;
    private final double VHANDOVER_PAN_ANGLE = -4.0;


    // for v slider drop
    private double VPREDROP_TILT_ANGLE = 10;
    private double VDROP_TILT_ANGLE = 65;
    private double VDROP_LEN = 0.01;
    private static double VDROP_RISE_LEN = 0.04;
    private double VDROP_PAN_ANGLE = 36.64;



    private int pickLevel = 5;

    double vcheckPos = 0;
    double hCheckPos = 0;
    ElapsedTime timer = new ElapsedTime();

    int parkingPos = 1;  // for parking location, by camera
    boolean foundPole = false;
    boolean foundCone = false;
    double deltaDis = 0.00;
    boolean getCone = false;
    @Override
    public void runOpMode(){
        cameraObjectDetector = new CameraObjectDetector(this, hardwareMap,telemetry);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        chassis = new ChassisSystem(hardwareMap, telemetry);


        robotPos = new RobotPosition(STARTX,STARTY,STARTA);
        odometry = new Odometry(this, hardwareMap, chassis,robotPos, telemetry, true);

        sleep(1000);
        telemetry.addLine("odometry is setup");
        telemetry.update();

        //mecanumDrive = new MecanumXYTurnDriveLib(this, chassis, odometry,telemetry);

        chassisController = new MecanumDriveLib(this, odometry,chassis, telemetry);

        hSliderSystem = new HSliderSystem(this, hardwareMap, telemetry, true);
        sleep(1000);

        vSliderSystem = new VSliderSystem(this, hardwareMap, telemetry, true);

        writeSDCardFile(robotPos);

        telemetry.addData("Working Mode", "waiting for start");
        robotPos = odometry.getRobotPosition();
        telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
        telemetry.update();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        hSliderSystem.enableTouchCheck();
        vSliderSystem.enableTouchCheck();

        timer.reset();
        pickLevel = 5;
        parkingPos = cameraObjectDetector.identifyTeamObject();// will take some time here, 3 frame only

        if (parkingPos == 1){
            parkPosition.x = STARTX + 0.67;
            parkPosition.y = STARTY + 0.7;
            parkPosition.angle = 0;
        }else if(parkingPos == 2){
            parkPosition.x = STARTX + 0.67;
            parkPosition.y = STARTY + 0.1;
            parkPosition.angle = 0;
        }else if(parkingPos == 3){
            parkPosition.x = STARTX + 0.67;
            parkPosition.y = STARTY - 0.55;
            parkPosition.angle = 0;
        }
        writeSDCardFile(parkPosition);

        vSliderSystem.setClawAngle(vSliderSystem.CLAW_CLOSE_ANGLE);
        vSliderSystem.sliderLenCtrl(VDROP_LEN, 0.5);
        vSliderSystem.setTiltAngle(-20);
        // reset claw to right position first, only for this tele demo program

        robotPos = odometry.getRobotPosition();

        //go strafe right and to medium high junction

        chassisController.p2pDrive(new RobotPosition(STARTX + 0.08,STARTY + 0.23,STARTA + 0),targetVel,turnVel,0.1,10,0,false,2000);
        while(opModeIsActive() && !chassisController.p2pIsSettled()){
            sleep(10);
        }
        chassisController.stopP2PTask();
        vSliderSystem.setPanAngle(VDROP_PAN_ANGLE);
        targetVel = 0.6;
        turnVel = 0.4;
        chassisController.p2pDrive(new RobotPosition(targetX,targetY,0),targetVel,turnVel,0.1,15,0,false,2500);
        while(opModeIsActive() && !chassisController.p2pIsSettled()){
            sleep(10);
        }
        chassisController.stopP2PTask();
        hSliderSystem.setTiltAngle(0);  //pre tilt the hslide
        hSliderSystem.setPanAngle(0);  //pre pan the hslide

        turnVel = 0.5;
        chassisController.p2pDrive(new RobotPosition(targetX,targetY,targetA),targetVel,turnVel,0.01,1,0,true,2500);
        while(opModeIsActive() && !chassisController.p2pIsSettled()){
            sleep(10);
        }

        chassisController.stopP2PTask();

        //double angleDiff = targetA - odometry.getRobotPosition().angle;
        //HPICKUP_PAN_ANGLE = HPICKUP_PAN_ANGLE - angleDiff;

        if (Math.abs(robotPos.x - targetX) > 0.05 || Math.abs(robotPos.y - targetY) > 0.05 || Math.abs(robotPos.angle - targetA) < 10){
            //position is not right, something wrong(may crash with other robot), try again
            turnVel = 0.5;
            chassisController.p2pDrive(new RobotPosition(targetX,targetY,targetA),targetVel,turnVel,0.01,1,0,true,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled()){
                sleep(10);
            }

            chassisController.stopP2PTask();
        }

        hSliderGotoPickUpPos(pickLevel,false);

        /////////////////////////////////////////////////////////////////////////////////
        if (!foundPole){
            vSliderSystem.scanToFindJunction();
            ElapsedTime timeout = new ElapsedTime();
            while(opModeIsActive() && !vSliderSystem.isFoundJunction() && timeout.milliseconds() < 4000)
            {
                sleep(10);
            }

            // to find junction

            if(vSliderSystem.isFoundJunction())
            {
                vSliderSystem.stopScan();
                sleep(50);
                VDROP_PAN_ANGLE = vSliderSystem.getPanAngle();
                //chassisController.setP2PKi(0.2,0.4);
                //if need add this
                // moveRobotForJunction();
            }
            else {
                vSliderSystem.stopScan();
                vSliderSystem.setPanAngle(VDROP_PAN_ANGLE);
            }
            foundPole= true;
        }

        chassisController.stopRobot();
        chassisController.setHoldPositionTaskStart(odometry.getRobotPosition());
        ///////////////////////////////////////////////////////////////////////////////////////

        //telemetry.addData("pan angle", "%2.2f", VDROP_PAN_ANGLE);
        //telemetry.update();

        //pick up cone
        boolean finishFlag = false;
        getCone = false;
        int count = 5;
        robotPos = odometry.getRobotPosition();
        if (opModeIsActive()  ) {
            while (opModeIsActive() && !finishFlag && timer.milliseconds() < 26000) {
                vSliderDropCone();
                //suppose the hSlider already at position
                gotoHandOverPos();
                handOverCone();
                if (getCone){
                    pickLevel--;
                    getCone = false;
                }
                count --;
                hSliderGotoPickUpPos(pickLevel, false);
                vSliderGotoDropPos();
                if (count <= 0) {
                    finishFlag = true;
                }
            }

            vSliderDropCone();
        }
        chassisController.setHoldPositionTaskOver();

        vSliderSystem.setTiltAngle(0);
        vSliderSystem.setPanAngle(0);
        vSliderSystem.sliderLenCtrl(0.1, 1.0);

        hSliderSystem.sliderLenCtrl(0.01, 1.0);
        hSliderSystem.setTiltAngle(20);
        hSliderSystem.setPanAngle(0);
        hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
        sleep(500);
/////////////////////////////////



        //for medium pole
        turnVel = 0.7;
        if (parkingPos == 1 && opModeIsActive()) {
            chassisController.p2pDrive(new RobotPosition(STARTX + 1.1,STARTY + 0.1,STARTA + 0),targetVel,turnVel,0.05,8,0.1,false,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();

            chassisController.p2pDrive(new RobotPosition(STARTX + 0.67,STARTY + 0.1,STARTA + 0),targetVel,turnVel,0.05,8,0.1,false,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();

            chassisController.p2pDrive(new RobotPosition(parkPosition.x,parkPosition.y,parkPosition.angle ),targetVel,turnVel,0.01,1,0,true,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();


        }
        else if(parkingPos == 2 && opModeIsActive()){
            chassisController.p2pDrive(new RobotPosition(STARTX + 1.1,STARTY + 0.1,STARTA + 0),targetVel,turnVel,0.05,8,0.1,false,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();

            chassisController.p2pDrive(new RobotPosition(parkPosition.x,parkPosition.y,parkPosition.angle ),targetVel,turnVel,0.01,1,0,true,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();
        }else if(opModeIsActive()){
            chassisController.p2pDrive(new RobotPosition(STARTX + 1.1,STARTY + 0.1,STARTA + 0),targetVel,turnVel,0.05,8,0.1,false,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();

            chassisController.p2pDrive(new RobotPosition(STARTX + 0.67,STARTY + 0.1,STARTA + 0),targetVel,turnVel,0.05,8,0.1,false,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();

            chassisController.p2pDrive(new RobotPosition(parkPosition.x,parkPosition.y,parkPosition.angle ),targetVel,turnVel,0.01,1,0,true,2500);
            while(opModeIsActive() && !chassisController.p2pIsSettled() ){
                sleep(10);
            }
            chassisController.stopP2PTask();
        }



        updateTelemetry();

        //finish, stop all
        chassisController.stopAllThread();

        chassis.stopRobot();
        //write current position, may only use with tracking wheel
        robotPos = odometry.getRobotPosition();
        writeSDCardFile(robotPos);

        odometry.stopThread();
        ////////////////////////////
    }

    private void moveRobotForJunction()
    {
        double junctionDiff = (vSliderSystem.middleSensor.getDistance(DistanceUnit.MM) - 150) / 1000;  // 20mm get from testing

        junctionDiff = junctionDiff + Math.signum(junctionDiff) * 0.015;

        robotPos = odometry.getRobotPosition();

        if (Math.abs(junctionDiff) > 0.01 && Math.abs(junctionDiff) < 0.15) {
            double theta = robotPos.angle - vSliderSystem.getPanAngle();
            double x = 1.0 * junctionDiff * Math.cos(Math.toRadians(theta)) + robotPos.x;
            double y = 1.0 * junctionDiff * Math.sin(Math.toRadians(theta)) + robotPos.y;
            chassisController.setMinPower(0.2,0.2,0.2);
            chassisController.p2pDrive(new RobotPosition(x, y, robotPos.angle), 1.0, 0.5, 0.01, 1, 0, true, 2500);
            while (opModeIsActive() && !chassisController.p2pIsSettled()) {
                sleep(10);
            }
            chassisController.stopP2PTask();
            chassisController.setMinPower(0.05,0.05,0.05);
        }
    }

    public void updateTelemetry()
    {
        telemetry.addData("Position", "%.2f %.2f %.2f", robotPos.x, robotPos.y, robotPos.angle);
        telemetry.addData("Timer:", "%f", timer.milliseconds());
        telemetry.addData("ParkingPos", " %d", parkingPos);
        telemetry.addData("FoundJunctionStatus", vSliderSystem.isFoundJunction());
        telemetry.update();
    }

    private void hSliderGotoPickUpPos(int level, boolean wait)
    {

        if (opModeIsActive()) {
            ElapsedTime timeOut = new ElapsedTime();
            if (level == 5) {
                //h5
                hCheckPos = HPICKUP_LEN + deltaDis;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-77.18);
                hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 4) {
                //h4
                hCheckPos = HPICKUP_LEN + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-85.38);
                hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 3) {
                //h3
                hCheckPos = HPICKUP_LEN + deltaDis;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-90.74);
                hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 2) {
                //h2
                hCheckPos = HPICKUP_LEN  + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-99);
                hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            } else if (level == 1) {
                //h1
                hCheckPos = HPICKUP_LEN + deltaDis;;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(-103);
                hSliderSystem.setPanAngle(HPICKUP_PAN_ANGLE);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);

            } else if (level == 0) {
                //h1
                hCheckPos = 0.12;
                hSliderSystem.sliderLenCtrl(hCheckPos, 1.0);
                hSliderSystem.setTiltAngle(90);
                hSliderSystem.setPanAngle(0);
                hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            }
            if (wait) {
                if (level != 0) {
                    timeOut.reset();
                    while (opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hCheckPos) > 0.01 && timeOut.milliseconds() < 1500) {
                        sleep(10);
                    }
                    sleep(300);
                }
            }
        }
    }

    public void gotoHandOverPos(){

        // pick up cone
        if (opModeIsActive()) {
            ElapsedTime timeOut = new ElapsedTime();
            vSliderSystem.sliderLenCtrl(VHANDOVER_LEN, 1.0);
            vSliderSystem.setTiltAngle(VHANDOVER_TILT_ANGLE);
            vSliderSystem.setPanAngle(VHANDOVER_PAN_ANGLE);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);

            // take the cone first
            timeOut.reset();
            while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - hCheckPos ) > 0.02 && timeOut.milliseconds() < 2500)
            {
                sleep(10);
            }

            if (!foundCone) {
                foundCone = true;
                hSliderSystem.scanToFindCone();
                timeOut.reset();
                while(opModeIsActive() && !hSliderSystem.isFoundCone() && timeOut.milliseconds() < 3000){
                    sleep(10);
                }


                if(hSliderSystem.isFoundCone()){
                    HPICKUP_PAN_ANGLE = hSliderSystem.getPanAngle();
                    hSliderSystem.stopScan();
                }
                hSliderSystem.stopScan();
                //adjust distance here
                //push and then move back
                hSliderSystem.slowMoveSliderMtr(0.5);
                double encoderReading = hSliderSystem.getSliderMtrEncoder();
                double lastEncoderReading = encoderReading;
                sleep(100);   //350
                timeOut.reset();
                while(opModeIsActive() && timeOut.milliseconds() < 1500){
                    encoderReading = hSliderSystem.getSliderMtrEncoder();
                    encoderReading = hSliderSystem.getSliderMtrEncoder();  // read twice to make sure it is the latest reading
                    if (Math.abs(encoderReading - lastEncoderReading) < 5){
                        break;
                    }
                    lastEncoderReading = encoderReading;
                    sleep(50);
                }

                double targetLen = hSliderSystem.getSliderLen();  // need update sensor reading first
                targetLen = hSliderSystem.getSliderLen() - 0.06;
                HPICKUP_LEN = targetLen;
                hSliderSystem.sliderLenCtrl(targetLen, 0.5);
                timeOut.reset();
                while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() -targetLen) > 0.01 && timeOut.milliseconds() < 1000){
                    sleep(10);
                }

            }

            hSliderSystem.setClawAngle(HSliderSystem.CLAW_CLOSE_ANGLE);
            sleep(300);
            //using sensor to find out getting cone or not
            if (hSliderSystem.leftSensor.getDistance(DistanceUnit.MM) < 80 || hSliderSystem.rightSensor.getDistance(DistanceUnit.MM)  < 80){
                getCone = true;
            }
            hSliderSystem.setTiltAngle(HHANDOVER_MIDDLE_TILT_ANGLE);
            sleep(200);

            hSliderSystem.sliderLenCtrl(HSLIDER_HANDOVER_LEN, 1.0);
            hSliderSystem.setPanAngle(HHANDOVER_PAN_ANGLE);
            timeOut.reset();
            while(opModeIsActive() && Math.abs(hSliderSystem.getSliderLen() - HSLIDER_HANDOVER_LEN) > 0.05 && timeOut.milliseconds() < 3000){
               sleep(10);
            }

            //  sleep(200);


            hSliderSystem.setTiltAngle(HHANDOVER_TILT_ANGLE);
            sleep(300);


            hSliderSystem.setClawAngle(HSliderSystem.CLAW_OPEN_ANGLE);
            vSliderSystem.sliderLenCtrl(VHANDOVER_LEN, 1.0);
            timeOut.reset();
            while(opModeIsActive() && Math.abs(vSliderSystem.getSliderLen() - VHANDOVER_LEN ) > 0.02 && timeOut.milliseconds() < 3000){
                sleep(10);
            }
            sleep(100);
            hSliderSystem.sliderLenCtrl(HPICKUP_LEN, 1.0);

        }
    }

    public void handOverCone(){
        if(opModeIsActive()) {
            sleep(100);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_CLOSE_ANGLE);
            sleep(200);
        }
    }

    public void vSliderGotoDropPos(){
        int vSliderHeight = 0;
        if(opModeIsActive()) {

            ElapsedTime timeOut = new ElapsedTime();
            vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
            sleep(200);
            vSliderSystem.setPanAngle(VDROP_PAN_ANGLE);
            vSliderSystem.sliderLenCtrl(VDROP_LEN, 1.0);
            timeOut.reset();
            while(Math.abs(vSliderSystem.getSliderLen() - VDROP_LEN) > 0.02  && timeOut.milliseconds() < 1500){
                sleep(10);
            }

            sleep(100);

        }
    }

    public void vSliderDropCone()
    {
        if(opModeIsActive()) {
            ElapsedTime timeOut = new ElapsedTime();
            foundPole = true;
            vSliderSystem.sliderLenCtrl( VDROP_LEN + VDROP_RISE_LEN, 0.75);
            vSliderSystem.setTiltAngle(VDROP_TILT_ANGLE);
            sleep(200);
            vSliderSystem.setClawAngle(VSliderSystem.CLAW_OPEN_ANGLE);
            sleep(150);
            vSliderSystem.sliderLenCtrl(VDROP_LEN , 1.0);
            vSliderSystem.setTiltAngle(VPREDROP_TILT_ANGLE);
            sleep(100);
        }
    }

    private void writeSDCardFile(RobotPosition currentPos)
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        dir.mkdirs();
        File file = new File(dir, "parkingPos.txt");
        try {
            FileOutputStream f = new FileOutputStream(file, false);
            final PrintStream printStream = new PrintStream(f);


            printStream.println(Double.toString(currentPos.x));
            printStream.println(Double.toString(currentPos.y));
            printStream.println(Double.toString(currentPos.angle));
            printStream.println("L");   // Right
            printStream.close();
            f.close();
        }
        catch(Exception e){

        }

    }



}