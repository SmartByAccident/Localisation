package org.firstinspires.ftc.teamcode.hardware.camera;


import static org.firstinspires.ftc.teamcode.utilities.TeamElement.CENTER;
import static org.firstinspires.ftc.teamcode.utilities.TeamElement.LEFT;
import static org.firstinspires.ftc.teamcode.utilities.TeamElement.RIGHT;

import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.teamcode.utilities.TeamElement;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import org.firstinspires.ftc.teamcode.utilities.FieldColor;
public class MatDetect implements VisionProcessor{

  public long count;
  public int centerCount;
  public int rightCount;

  public TeamElement getPosition(){

    if(centerCount > 200){
      return CENTER;
    }

    if(rightCount > 200){
      return RIGHT;
    }

    return LEFT;
   
  }


  public java.lang.Object processFrame(org.opencv.core.Mat image,long captureTime){

    count = 0;
    centerCount = getPixelCountFor(image,CENTER);
    rightCount = getPixelCountFor(image, RIGHT);
    return image;

  }

  private int getPixelCountFor(Mat image, TeamElement area) {
    int count = 0;
    for(int rowIndex = area.topX; rowIndex< area.bottomX;rowIndex++){
      for(int columnIndex = area.topY; columnIndex< area.bottomY; columnIndex++){
        Scalar color = new Scalar(image.get(columnIndex, rowIndex));
        double blueColor = color.val[2];
        double greenColor = color.val[1];
        double redColor = color.val[0];
        if (FieldColor.getActive() == FieldColor.BLUE){
          if(redColor < 100 && (greenColor<140) && (blueColor>120) ){
            count = count +1;
          }
        }else{
          if(redColor > 100 && (greenColor<100) && (blueColor<100) ){
            count = count +1;
          }
        }
      }
    }
    return count;
  }


  public void init(int width,int height ,org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration){

  }

  public void onDrawFrame(android.graphics.Canvas canvas,int width,int height,float scale,float density,java.lang.Object bmp){

    Paint paint = new Paint();
    paint.setStyle(Paint.Style.STROKE);
    paint.setColor(Color.BLACK);
    for(int i = 10; i<width; i +=10){
      canvas.drawLine(i*scale,0f,i*scale,height*scale,paint);
      
    }
    for(int i = 10; i<height; i +=10){
      canvas.drawLine(0f,i*scale,width*scale,i*scale,paint);
      
    }
    paint.setStrokeWidth(3);
    paint.setColor(Color.WHITE);
    canvas.drawRect(CENTER.topX *scale, CENTER.topY*scale,CENTER.bottomX*scale,CENTER.bottomY *scale,paint);
    canvas.drawRect(RIGHT.topX *scale, RIGHT.topY*scale,RIGHT.bottomX*scale,RIGHT.bottomY*scale,paint);
    
  }
}