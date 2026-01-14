package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import android.util.Size;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "Vision Testing", group = "Util")
public class VisionTesting extends OpMode {


    JoinedTelemetry joinedTelemetry;


    AprilTagProcessor ATProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .build();
    VisionPortal visionPortal;

    ArrayList<AprilTagDetection> detections = new ArrayList<>();


    AprilTagDetection detection;

    @Override
    public void init() {

        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(ATProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


    }

    @Override
    public void loop() {

        detections = ATProcessor.getDetections();

        for (int i = 0; i < detections.size() - 1; i++) {

            detection = detections.get(i);

            telemetry.addData("Detection # ", i);
            telemetry.addData("ID # ", detections.get(i).id);
            telemetry.addData("Distance To AprilTag ", detections.get(i).ftcPose.range);
            telemetry.addData("Change In Angle To Face AprilTag ", detections.get(i).ftcPose.bearing);
            telemetry.addLine();
            if (UniConstants.obeliskIDs.contains(detection.id)) {
                telemetry.addLine("If ID is Obelisk: ");
                telemetry.addData("Pattern String ", getPatternAsString(obeliskTargetPattern(detection.id)));
                telemetry.addData("Pattern Array ", obeliskTargetPattern(detection.id));

            }

            telemetry.addLine();

        }

        telemetry.update();

    }


    public ArrayList<IntakeSortingSubsystem.Slot.slotState> obeliskTargetPattern(int ID) {
        switch (ID) {
            case 21:
                return new ArrayList<>(List.of(IntakeSortingSubsystem.Slot.slotState.GREEN, IntakeSortingSubsystem.Slot.slotState.PURPLE, IntakeSortingSubsystem.Slot.slotState.PURPLE));
            case 22:
                return new ArrayList<>(List.of(IntakeSortingSubsystem.Slot.slotState.PURPLE, IntakeSortingSubsystem.Slot.slotState.GREEN, IntakeSortingSubsystem.Slot.slotState.PURPLE));
            case 23:
                return new ArrayList<>(List.of(IntakeSortingSubsystem.Slot.slotState.PURPLE, IntakeSortingSubsystem.Slot.slotState.PURPLE, IntakeSortingSubsystem.Slot.slotState.GREEN));
            default:
                return new ArrayList<>(List.of(null, null, null));
        }
    }

    public String getPatternAsString(ArrayList<IntakeSortingSubsystem.Slot.slotState> pattern) {
        StringBuilder returnString = new StringBuilder();

        for (IntakeSortingSubsystem.Slot.slotState state : pattern) {
            switch (state) {
                case GREEN:
                    returnString.append("G");
                    break;
                case PURPLE:
                    returnString.append("P");
                    break;
            }

        }

        return returnString.toString();

    }


}


