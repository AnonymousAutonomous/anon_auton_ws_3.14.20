PImage webImg;
float threshold = 175;
float topBandWidth = 120;
float sideBandWidth = 180;
float side_percent_threshold = 0.20;
float top_percent_threshold = 0.20;

int width = 640;
int height = 480;

Handle[] handles;
// threshold, topBandWidth, sideBandWidth, top_percent_threshold, side_percent_threshold
float[] variables = {175, 120, 180, 0.20, 0.20};


void setup() {
  size(640,600);
  background(80);
  
  // Load image from a web server
  webImg = loadImage("test.JPG");
  webImg.resize(width, height);

   handles = new Handle[5];
  int hsize = 10;
  handles[0] = new Handle(width/2, 500+0*15, 50-hsize/2, 10, handles, 0, 255, 0, variables[0]);
  handles[1] = new Handle(width/2, 500+1*15, 50-hsize/2, 10, handles, 0, height, 1, variables[1]);
  handles[2] = new Handle(width/2, 500+2*15, 50-hsize/2, 10, handles, 0, width / 2, 2, variables[2]);
  handles[3] = new Handle(width/2, 500+3*15, 50-hsize/2, 10, handles, 0, 1, 3, variables[3]);
  handles[4] = new Handle(width/2, 500+4*15, 50-hsize/2, 10, handles, 0, 1, 4, variables[4]);

  
}

void draw() {
  threshold = variables[0];
  topBandWidth = variables[1];
  sideBandWidth = variables[2];
  top_percent_threshold = variables[3];
  side_percent_threshold = variables[4];


  stroke(0, 0, 0);
  strokeWeight(1);
  background(80);
  image(webImg, 0, 0);
  loadPixels();
  for (int i = 0; i < width * height; i++) {
    if (brightness(pixels[i]) > threshold) {
      pixels[i] = color(255, 0, 0);
    }
  }
  updatePixels();
  
   for (int i = 0; i < handles.length; i++) {
    handles[i].update();
    handles[i].display();
  }
  
  textSize(15);
  text("threshold: " + str(threshold), 10, 500+0*15); 
  text("top band width: " + str(topBandWidth), 10, 500+1*15); 
  text("side band width: " + str(sideBandWidth), 10, 500+2*15); 
  text("top percent threshold: " + str(top_percent_threshold), 10, 500+3*15); 
  text("side percent threshold: " + str(side_percent_threshold), 10, 500+4*15); 


  
  stroke(0, 0, 0);
  strokeWeight(5);
  line(0, topBandWidth, width, topBandWidth);
  line(sideBandWidth, 0, sideBandWidth, height);
  line(width - sideBandWidth, 0, width - sideBandWidth, height);
  
  updateCells();
}

void updateCells() {
    double topCount = 0;
    double leftCount = 0;
    double rightCount = 0;
    double middleCount = 0;


  for (int i = 0; i < width * height; i++) {
    if(inTop(i))
        {
            if (brightness(pixels[i]) > threshold) {
              topCount += 1.0;
            }
        }
     if(inLeft(i))
     {
            if (brightness(pixels[i]) > threshold) {
              leftCount += 1.0;
            }
        }
        if(inRight(i))
     {
            if (brightness(pixels[i]) > threshold) {
              rightCount += 1.0;
            }
        }
        if(inMiddle(i))
     {
            if (brightness(pixels[i]) > threshold) {
              middleCount += 1.0;
            }
        }
  }

    double leftAverage = leftCount/ (height*sideBandWidth);
    double rightAverage = rightCount / (height*sideBandWidth);
    double topAverage = topCount / (width*topBandWidth);
    double middleAverage = middleCount / (height * width);
    
          stroke(0, 255, 0);
          strokeWeight(10);

    if (leftAverage > side_percent_threshold) {
      line(sideBandWidth, 0, sideBandWidth, height);
    }
    
    if (rightAverage > side_percent_threshold) {
  line(width - sideBandWidth, 0, width - sideBandWidth, height);
      }
    
    if (topAverage > side_percent_threshold) {
  line(0, topBandWidth, width, topBandWidth);
    }
    
}

boolean inTop(int counter)
{
    return counter < (width*topBandWidth);
}

boolean inLeft(int counter)
{
    return (counter % width) < sideBandWidth;
}

boolean inRight(int counter)
{
    return (counter % width) > (width - sideBandWidth);
}

boolean inMiddle(int counter)
{
    return !(inTop(counter) || inLeft(counter) || inRight(counter));
}



void mouseReleased()  {
  for (int i = 0; i < handles.length; i++) {
    handles[i].releaseEvent();
  }
}


class Handle {
  
  int x, y;
  int boxx, boxy;
  int stretch;
  int size;
  float min;
  float max;
  boolean over;
  boolean press;
  boolean locked = false;
  boolean otherslocked = false;
  int index; 
  Handle[] others;

  
  Handle(int ix, int iy, int il, int is, Handle[] o, float imin, float imax, int iindex, float startval) {
    x = ix;
    y = iy;
    stretch = il;
    size = is;
    boxx = x+stretch - size/2;
    boxy = y - size/2;
    others = o;
    min = imin;
    max = imax;
    index = iindex;
    stretch = lock(int((startval / (max - min)) * 100), 0, 100);
  }
  
  void update() {
    boxx = x+stretch;
    boxy = y - size/2;
    
    for (int i=0; i<others.length; i++) {
      if (others[i].locked == true) {
        otherslocked = true;
        break;
      } else {
        otherslocked = false;
      }  
    }
    
    if (otherslocked == false) {
      overEvent();
      pressEvent();
    }
    
    if (press) {
      stretch = lock(mouseX-width/2-size/2, 0, 100);
    }
  }
  
  void overEvent() {
    if (overRect(boxx, boxy, size, size)) {
      over = true;
    } else {
      over = false;
    }
  }
  
  void pressEvent() {
    if (over && mousePressed || locked) {
      press = true;
      locked = true;
    } else {
      press = false;
    }
  }
  
  void releaseEvent() {
    locked = false;
    println(boxx - x);
    println(size);

    float frac = (boxx - x) / 100.0;
    println(boxx - x);
    println(frac);
    println(min + (frac * (max - min)));
    variables[index] = min + (frac * (max - min));
  }
  
  void display() {
    line(x, y, x+stretch, y);
    fill(255);
    stroke(0);
    rect(boxx, boxy, size, size);
    if (over || press) {
      line(boxx, boxy, boxx+size, boxy+size);
      line(boxx, boxy+size, boxx+size, boxy);
    }

  }
}

boolean overRect(int x, int y, int width, int height) {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    return true;
  } else {
    return false;
  }
}

int lock(int val, int minv, int maxv) { 
  return  min(max(val, minv), maxv); 
} 
