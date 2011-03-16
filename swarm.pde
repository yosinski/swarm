import processing.opengl.*;
import peasy.*;

PeasyCam cam;

Flock flock;

final int ENV_X = 300;
final int ENV_Y = 300;
final int ENV_Z = 50;

float fov = PI/3.0;
float cameraZ = (height/2.0) / tan(fov/2.0);

void setup() {
  size(1280,720,OPENGL);
  frameRate(24);
  
  cam = new PeasyCam(this, 200);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(500);
  perspective(fov, float(width)/float(height), 2, 10000.0);
  
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 150; i++) {
    flock.addBoid(new Boid(new PVector(ENV_X/2, ENV_Y/2, ENV_Z/2), 3.0, 0.05));
  }
}

void draw() {
  background(50);
  float camLen = 500;
  camera(sin(radians(frameCount))*camLen+ENV_X/2, cos(radians(frameCount))*camLen+ENV_X/2,camLen*.8,ENV_X/2,ENV_X/2,0, 0.0,0.0,-1);
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
/*void mousePressed() {
  flock.addBoid(new Boid(new PVector(mouseX,mouseY),2.0f,0.05f));
}
*/












