import processing.opengl.*;
import peasy.*;

PeasyCam cam;

Flock flock;

final int ENV_X = 300;
final int ENV_Y = 300;
final int ENV_Z = 300;

float fov = PI/3.0;
float cameraZ = (height/2.0) / tan(fov/2.0);

void setup() {
  size(1280,720,OPENGL);
  frameRate(24);

  cam = new PeasyCam(this, 200);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(1000);
  perspective(fov, float(width)/float(height), 2, 10000.0);
  float camLen = 500;
  camera(sin(radians(frameCount))*camLen+ENV_X/2, cos(radians(frameCount))*camLen+ENV_X/2,camLen*.8,ENV_X/2,ENV_X/2,0, 0.0,0.0,-1);

  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 300; i++) {
    //flock.addBoid(new Boid(new PVector(ENV_X/2, ENV_Y/2, ENV_Z/2), 3.0, 0.05));
      float windowSize = 50;
    flock.addBoid(new Boid(new PVector(random(ENV_X/2-windowSize, ENV_X/2+windowSize), random(ENV_Y/2-windowSize, ENV_Y/2+windowSize),  random(ENV_Z/2-windowSize, ENV_Z/2+windowSize)), 1.0, 0.05, true));
  }
}

void draw() {
  background(50);
  float camLen = 500;
  //camera(sin(radians(frameCount))*camLen+ENV_X/2, cos(radians(frameCount))*camLen+ENV_X/2,camLen*.8,ENV_X/2,ENV_X/2,0, 0.0,0.0,-1);
  flock.run();

  pushMatrix();
  translate(ENV_X/2,ENV_Y/2,ENV_Z/2);
  stroke(150);
  strokeWeight(.1);
  noFill();
  box(ENV_X,ENV_Y,ENV_Z);
  popMatrix();
}

// Add a new boid into the System
/*
void mousePressed() {
 flock.addBoid(new Boid(new PVector(mouseX,mouseY),2.0f,0.05f));
 }
 */











