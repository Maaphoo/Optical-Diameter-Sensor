  
//Diameter sensor UI by Matt Rogge August 25, 2016
//todo
// - Draw vertical lines where the diameter is 
// - receive diameter measurements and edges
import processing.serial.*; 
import org.gicentre.utils.stat.*;    // For chart classes.

 
Serial myPort;    // The serial port
PFont myFont;     // The display font
String inString;  // Input string from serial port
int linefeed = 10;      // ASCII linefeed 
float[] vals = new float[256];

//Chart Variables
XYChart lineChart;
float[] dataX = new float[256];//The data for the x axis
float[] dataY = new float[256];;//The data for the y axis



void setup() {
  frameRate(60);
  size(1000,500);
  //Serial port setup
  // List all the available serial ports: 
  printArray(Serial.list());
  
  // Open the port that the arduino uses.
  myPort = new Serial(this, Serial.list()[3], 115200); 
  myPort.bufferUntil(linefeed); 
  
  //Chart Setup

  textFont(createFont("Arial",10),10);
 
  // Both x and y data set here.  
  lineChart = new XYChart(this);
  dataX = new float[256];
  dataY = new float[256];
  for (int i=0;i<dataX.length;i++){
      dataX[i] = i*0.0635;
    }
  lineChart.setData(dataX,dataY);
   
  // Axis formatting and labels.
  lineChart.showXAxis(true); 
  lineChart.showYAxis(true); 
  lineChart.setMinY(0);
  lineChart.setMaxY(1024);
  
  // Symbol colours
  lineChart.setPointColour(color(180,50,50,100));
  lineChart.setPointSize(5);
  lineChart.setLineWidth(2);
  lineChart.setYAxisLabel("Intensity");
} 
 
void draw() { 

  background(255);
  textSize(9);
  //text("received: " + inString, 10,50);
  lineChart.setData(dataX,dataY);
    //for (int i = 0; i<dataY.length;i++){
    //  //dataY[i] = float(vals[i]);
    //  print(dataY[i]);
    //  print(" ");
    //}
    //println();
    //for (int i = 0; i<dataY.length;i++){
    //  print(dataX[i]);
    //  print(" ");
    //}    
    //println();
    //println();
  lineChart.draw(15,50,width-30,height-80);
   
  // Draw a title over the top of the chart.
  fill(120);
  textSize(20);
  text("Diameter Sensor", 70,30);
  textSize(11);
  text("Intensity measured at each pixel", 70,45);
} 
 
void serialEvent(Serial p) { 
  try{
  inString = p.readString(); 
  if (vals != null){
    //print("before split tokens: ");
    //println(dataY.length);  
    
    vals = float(split(inString, ' '));//splitTokens(inString);
    //println("Contents of vals");
    //for (int i=0; i<vals.length; i++){
    //  print(vals[i]);
    //  print(" ");
    //}
    //println("");
    
    //print("after split tokens: ");
    //println(vals.length);
    
    if (vals.length > 256){
      //println("Shortening");
      for (int i = 0; i<vals.length-256;i++){
        dataY = shorten(vals);
      }
    } else if (vals.length < 256) {
      //println("Lengthening");
      for (int i = 0;i<256-vals.length;i++){
        dataY = append(vals, 0.0);
      }
    }
    //print("final length: ");
    //println(dataY.length);
    //println("Contents of dataY");
    //for (int i=0; i<dataY.length; i++){
    //  print(dataY[i]);
    //  print(" ");
    //}
    //for (int i = 0; i<vals.length;i++){
    //  dataY[i] = float(vals[i]);
    //}
  }
  }catch (Exception e) {
    println("error");
  }
} 