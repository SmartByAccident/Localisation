package org.firstinspires.ftc.teamcode.utilities;

public enum TeamElement {
    /* Blue Main
    CENTER (130,200),
    RIGHT(420,230),
    LEFT(0,0);*/
    /*Red Main
    CENTER (220,190),
    RIGHT(500,220),
    LEFT(0,0);*/
    /*Blue Max
    CENTER (140,190),
    RIGHT(440,220),
    LEFT(0,0);*/
    CENTER(220,190),
      RIGHT(510,210),
      LEFT(0,0);
    public int topX;
    public int topY;

    public int bottomX;
    public int bottomY;
    
    private int topXValue;
    private int topYValue;

    public void set(int topX, int topY)
        {
            topXValue = topX;
            topYValue = topY;

            this.topX = topX;
            this.topY = topY;
            this.bottomX = topX+40;
            this.bottomY = topY+40;
        }


    TeamElement(int topX, int topY) {
        set(topX,topY);
    }
    
    
    
    

    
}
