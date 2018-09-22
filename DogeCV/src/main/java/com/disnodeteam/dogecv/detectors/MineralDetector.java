package com.disnodeteam.dogecv.detectors;

import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MineralDetector extends OpenCVPipeline {

    public enum MineralOrder {

        R_G_R,
        G_R_R,
        R_R_G,
        UNKNOWN

    }

    public enum MineralDetectionMode {

        PERFECT_AREA, MAX_AREA

    }

    public enum MineralDetectionSpeed {

        VERY_FAST, FAST, BALANCED, SLOW, VERY_SLOW

    }

    public MineralDetectionMode detectionMode = MineralDetectionMode.MAX_AREA;
    public double downScaleFactor = 0.4;
    public double perfectRatio = 1;
    public boolean rotateMat = false;
    public MineralDetectionSpeed speed = MineralDetectionSpeed.BALANCED;
    public double perfectArea = 6500;
    public double areaWeight = 0.05;
    public double minArea = 700;
    public double ratioWeight = 15;
    public double maxDifference = 10;
    public boolean debugContours = false;
    public DogeCVColorFilter colorFilterYellow = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
    public DogeCVColorFilter colorFilterRed = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
    public DogeCVColorFilter colorFilterRed2 = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);

    private MineralOrder currentOrder = MineralOrder.UNKNOWN;
    private MineralOrder lastOrder = MineralOrder.UNKNOWN;

    private Mat workingMat = new Mat();
    private Mat blurredMat = new Mat();
    private Mat maskYellow = new Mat();
    private Mat maskRed = new Mat();
    private Mat maskRed2 = new Mat();
    private Mat hiarchy = new Mat();

    private Size newSize = new Size();

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        Size initSize = rgba.size();
        newSize = new Size(initSize.width * downScaleFactor, initSize.height * downScaleFactor);
        rgba.copyTo(workingMat);

        Imgproc.resize(workingMat, workingMat, newSize);

        if (rotateMat) {

            Mat tempBefore = workingMat.t();

            Core.flip(tempBefore, workingMat, -1);

            tempBefore.release();

        }

        Mat yellowConvert = workingMat.clone();
        Mat redConvert = workingMat.clone();
        Mat red2Convert = workingMat.clone();

        colorFilterYellow.process(yellowConvert, maskYellow);
        colorFilterRed.process(redConvert, maskRed);
        colorFilterRed2.process(red2Convert, maskRed2);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(maskYellow, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat, contoursYellow, -1, new Scalar(230, 70, 70), 2);
        Rect chosenYellowRect = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for (MatOfPoint c : contoursYellow) {

            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            Rect rect = Imgproc.boundingRect(points);


            double area = Imgproc.contourArea(c);
            double areaDifference = 0;

            switch (detectionMode) {

                case MAX_AREA:
                    areaDifference = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDifference = Math.abs(perfectArea - area);
                    break;
            }


            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + (w / 2), y + (h / 2));


            double cubeRatio = Math.max(Math.abs(h / w), Math.abs(w / h));
            double ratioDifference = Math.abs(cubeRatio - perfectRatio);

            double finalDifference = (ratioDifference * ratioWeight) + (areaDifference * areaWeight);

            if (finalDifference < chosenYellowScore && finalDifference < maxDifference && area > minArea) {
                chosenYellowScore = finalDifference;
                chosenYellowRect = rect;
            }

            if (debugContours && area > 100) {
                Imgproc.circle(workingMat, centerPoint, 3, new Scalar(0, 255, 255), 3);
                Imgproc.putText(workingMat, "Area: " + area, centerPoint, 0, 0.5, new Scalar(0, 255, 255));
            }


        }

        List<MatOfPoint> contoursRed = new ArrayList<>();

        Imgproc.findContours(maskRed, contoursRed, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat, contoursRed, -1, new Scalar(70, 130, 230), 2);
        Rect chosenRedRect = null;
        double chosenRedScore = Integer.MAX_VALUE;

        for (MatOfPoint c : contoursRed) {

            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            Rect rect = Imgproc.boundingRect(points);


            double area = Imgproc.contourArea(c);
            double areaDifference = 0;

            switch (detectionMode) {
                case MAX_AREA:
                    areaDifference = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDifference = Math.abs(perfectArea - area);
                    break;
            }

            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + (w / 2), y + (h / 2));

            double cubeRatio = Math.max(Math.abs(h / w), Math.abs(w / h));
            double ratioDifference = Math.abs(cubeRatio - 1);

            double finalDifference = (ratioDifference * ratioWeight) + (areaDifference * areaWeight);

            if (finalDifference < chosenRedScore && finalDifference < maxDifference && area > minArea) {
                chosenRedScore = finalDifference;
                chosenRedRect = rect;
            }


            if (debugContours && area > 100) {
                Imgproc.circle(workingMat, centerPoint, 3, new Scalar(0, 255, 255), 3);
                Imgproc.putText(workingMat, "Area : " + area, centerPoint, 0, 0.5, new Scalar(0, 255, 255));
            }

        }

        List<MatOfPoint> contoursRed2 = new ArrayList<>();

        Imgproc.findContours(maskRed2, contoursRed2, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat, contoursRed, -1, new Scalar(70, 130, 230), 2);
        Rect chosenRedRect2 = null;
        double chosenRedScore2 = Integer.MAX_VALUE;

        for (MatOfPoint c : contoursRed) {

            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            Rect rect = Imgproc.boundingRect(points);


            double area = Imgproc.contourArea(c);
            double areaDifference = 0;

            switch (detectionMode) {
                case MAX_AREA:
                    areaDifference = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDifference = Math.abs(perfectArea - area);
                    break;
            }

            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + (w / 2), y + (h / 2));

            double cubeRatio = Math.max(Math.abs(h / w), Math.abs(w / h));
            double ratioDifference = Math.abs(cubeRatio - 1);

            double finalDifference = (ratioDifference * ratioWeight) + (areaDifference * areaWeight);

            if (finalDifference < chosenRedScore2 && finalDifference < maxDifference && area > minArea) {
                chosenRedScore2 = finalDifference;
                chosenRedRect2 = rect;
            }

            if (chosenYellowRect != null) {
                Imgproc.rectangle(workingMat,
                        new Point(chosenYellowRect.x, chosenYellowRect.y),
                        new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                        new Scalar(255, 0, 0), 2);

                Imgproc.putText(workingMat,
                        "Yellow: " + String.format("%.2f", chosenYellowScore),
                        new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                        Core.FONT_HERSHEY_PLAIN,
                        1.3,
                        new Scalar(255, 0, 0),
                        2);
            }

        }
        if (chosenRedRect != null) {
            Imgproc.rectangle(workingMat,
                    new Point(chosenRedRect.x, chosenRedRect.y),
                    new Point(chosenRedRect.x + chosenRedRect.width, chosenRedRect.y + chosenRedRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(workingMat,
                    "Red: " + String.format("%.2f", chosenRedScore),
                    new Point(chosenRedRect.x - 5, chosenRedRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 0, 255),
                    2);
        }

        if (chosenRedRect2 != null) {
            Imgproc.rectangle(workingMat,
                    new Point(chosenRedRect2.x, chosenRedRect2.y),
                    new Point(chosenRedRect2.x + chosenRedRect2.width, chosenRedRect2.y + chosenRedRect2.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(workingMat,
                    "Red2: " + String.format("%.2f", chosenRedScore2),
                    new Point(chosenRedRect2.x - 5, chosenRedRect2.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 0, 255),
                    2);
        }

        if (chosenYellowRect != null && chosenRedRect != null && chosenRedRect2 != null) {
            if (chosenRedRect.x < chosenYellowRect.x && chosenYellowRect.x < chosenRedRect2.x) {
                currentOrder = MineralOrder.R_G_R;
                lastOrder = currentOrder;
            }
            if (chosenYellowRect.x < chosenRedRect.x && chosenRedRect.x < chosenRedRect2.x) {
                currentOrder = MineralOrder.G_R_R;
                lastOrder = currentOrder;
            } else {
                currentOrder = MineralOrder.R_R_G;
                lastOrder = currentOrder;
            }
        } else {
            currentOrder = MineralOrder.UNKNOWN;
        }

        Imgproc.putText(workingMat, "Result: " + lastOrder.toString(), new Point(10, newSize.height - 30), 0, 1, new Scalar(255, 255, 0), 1);
        Imgproc.putText(workingMat, "Current Track: " + currentOrder.toString(), new Point(10, newSize.height - 10), 0, 0.5, new Scalar(255, 255, 255), 1);

        redConvert.release();
        red2Convert.release();
        yellowConvert.release();
        Imgproc.putText(workingMat, "IronCV 1.1 Mineral: " + newSize.toString() + " - " + speed.toString() + " - " + detectionMode.toString(), new Point(5, 30), 0, 1.2, new Scalar(0, 255, 255), 2);


        return workingMat;
    }

    public MineralOrder getCurrentOrder() {
        return currentOrder;
    }

    public MineralOrder getLastOrder() {
        return lastOrder;
    }

}


