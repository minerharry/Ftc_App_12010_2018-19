package com.disnodeteam.dogecv.detectors;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.scoring.DogeCVScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.flip;
import static org.opencv.core.Core.transpose;

/**
 * Created by Victo on 9/10/2018.
 */

public abstract class DogeCVDetector extends OpenCVPipeline{

    public abstract Mat process(Mat input);
    public abstract void useDefaults();

    private List<DogeCVScorer> scorers = new ArrayList<>();
    private Size initSize;
    private Size adjustedSize;
    private Mat workingMat = new Mat();
    public double maxDifference = 10;
    public boolean isSideways = false; //added by Harrison Truscott; makes image function in landscape mode
    public double verticalMax = 1.0;
    public double verticalMin = 0.0; //these two added by Harrison Truscott; represent, in percentage (0.0-1.0), the amount of the image to be considered in scoring

    public DogeCV.DetectionSpeed speed = DogeCV.DetectionSpeed.BALANCED;
    public double downscale = 0.5;
    public Size   downscaleResolution = new Size(640, 480);
    public boolean useFixedDownscale = true;
    protected String detectorName = "DogeCV Detector";

    public DogeCVDetector(){

    }

    public void setSpeed(DogeCV.DetectionSpeed speed){
        this.speed = speed;
    }

    public void addScorer(DogeCVScorer newScorer){
        scorers.add(newScorer);
    }

    public double calculateScore(Mat input){
        double totalScore = 0;

        for(DogeCVScorer scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }



    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        initSize = rgba.size();

        if(useFixedDownscale){
            adjustedSize = downscaleResolution;
        }else{
            adjustedSize = new Size(initSize.width * downscale, initSize.height * downscale);
        }

        rgba.copyTo(workingMat);

        if(workingMat.empty()){
            return rgba;
        }
        Imgproc.resize(workingMat, workingMat,adjustedSize); // Downscale

        //Added Code: Minerharry (12010) - adapt for sideways cameras, larger field of view.

        if(isSideways)
        {
            transpose(workingMat,workingMat);
            flip(workingMat,workingMat,1);
        }
        Mat tempMat = process(workingMat);
        if(isSideways) {
            transpose(tempMat, tempMat);
            flip(tempMat,tempMat,0);
            transpose(workingMat,workingMat);
        }
        Imgproc.resize(tempMat,workingMat,getInitSize()); // Process and scale back to original size for viewing
        //Print Info
        Imgproc.putText(workingMat,"DogeCV 2018.2 " + detectorName + ": " + getAdjustedSize().toString() + " - " + speed.toString() ,new Point(5,30),0,0.5,new Scalar(0,255,255),2);

        return workingMat;
    }

    public Size getInitSize() {
        return initSize;
    }

    public Size getAdjustedSize() {
        return adjustedSize;
    }

    public void setAdjustedSize(Size size) { this.adjustedSize = size; }
}
