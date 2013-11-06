/*
 *  Csc 474, Computer Graphics
 *  Chris Buckalew, modified from Jason L. McKesson, Ed Angel, and others
 *
 * Simple vortex-tornado particle System
 *
 *------------------------------------------------------------------*/

#include <string>
#include <vector>
#include <stack>
#include <math.h>
#include <stdio.h>
//#include <GL/freeglut.h>
#include <GL/glut.h>
#include "../glm/glm.hpp"
#include "../glm/ext.hpp"
#include <fstream>
#include <sstream>
#include <exception>
#include <stdexcept>
#include <string.h>
#include <algorithm>
#include <string>
#include <vector>
#include "Sphere.cpp"
#include "util.cpp"
#define WIND 2
#define WIND2 5
#define OTHER 100

// prototypes and variables associated with the trackball viewer
void mouseCallback(int button, int state, int x, int y);
void mouseMotion(int x, int y);


bool explode = false;
bool trackballEnabled = true;
bool trackballMove = false;
bool trackingMouse = false;
bool redrawContinue = false;
bool zoomState = false;
bool shiftState = false;

int winWidth, winHeight;
float angle = 0.0, axis[3];

float lightXform[4][4] = {
   {1.0, 0.0, 0.0, 0.0},
   {0.0, 1.0, 0.0, 0.0},
   {0.0, 0.0, 1.0, 0.0},
   {0.0, 0.0, 0.0, 1.0}
};

float objectXform[4][4] = {
   {1.0, 0.0, 0.0, 0.0},
   {0.0, 1.0, 0.0, 0.0},
   {0.0, 0.0, 1.0, 0.0},
   {0.0, 0.0, 0.0, 1.0}
};

glm::vec3 winDir = glm::vec3(4,0,0);

float *objectXformPtr = (float *)objectXform;
float *lightXformPtr = (float *)lightXform;
float *trackballXform = (float *)objectXform;

// initial viewer position
static float modelTrans[] = {0.0f, 0.0f, -10.0f};

struct ProgramData
{
   // info for accessing the shaders
   GLuint theProgram;
   GLuint modelToWorldMatrixUnif;
   GLuint worldToCameraMatrixUnif;
   GLuint cameraToClipMatrixUnif;
   GLuint worldSpaceMoveMatrixUnif;
   GLuint diffuseColorUnif;
};

float nearClipPlane = 0.1f;
float farClipPlane = 1000.0f;

// buffers used to communicate with the GPU
GLuint vertexBufferObject;
GLuint indexBufferObject;
GLuint vao;

ProgramData Points;

ProgramData LoadProgram(const std::string &strVertexShader, const std::string &strFragmentShader)
{
   std::vector<GLuint> shaderList;

   shaderList.push_back(LoadShader(GL_VERTEX_SHADER, strVertexShader));
   shaderList.push_back(LoadShader(GL_FRAGMENT_SHADER, strFragmentShader));

   ProgramData data;
   data.theProgram = CreateProgram(shaderList);

   // the uniforms needed for the shaders
   data.modelToWorldMatrixUnif = glGetUniformLocation(data.theProgram, "modelToWorldMatrix");
   data.worldToCameraMatrixUnif = glGetUniformLocation(data.theProgram, "worldToCameraMatrix");
   data.cameraToClipMatrixUnif = glGetUniformLocation(data.theProgram, "cameraToClipMatrix");
   data.worldSpaceMoveMatrixUnif = glGetUniformLocation(data.theProgram, "worldSpaceMoveMatrix");
   data.diffuseColorUnif = glGetUniformLocation(data.theProgram, "diffuseColor");

   return data;
}

void InitializeProgram()
{
   // load and compile shaders, link the program and return it
   Points = LoadProgram("Points.vert", "Points.frag");
}

// set up the sphere statics in case there's going to be a sphere
// can remove these lines if no sphere
bool Sphere::sphereInitialized = false;
GLuint Sphere::sphereVao = vao;
GLuint Sphere::sphereVertexBufferObject = vertexBufferObject;
GLuint Sphere::sphereIndexBufferObject = indexBufferObject;

//---------------------------------------------------------
//  Particle system routines

const int NUM_PARTICLES =5000;
const float GRAVITY = 30.8;
const float DRAG = 0.47;
const float TIMESTEP = 0.001;

float mass[NUM_PARTICLES];
float pos[NUM_PARTICLES][3];
float vel[NUM_PARTICLES][3];
float acc[NUM_PARTICLES][3];
float force[NUM_PARTICLES][3];
float color[NUM_PARTICLES][4];
float startTime[NUM_PARTICLES];

void initializeParticleData(int i) {
   // initial values for all the particles
   mass[i] = 1.0;

   pos[i][0] = 0.0f;
   pos[i][1] = 0.0f;
   pos[i][2] = 0.0f;

   vel[i][0] = ((float) rand()/RAND_MAX) * 5.0f - 2.0f;
   vel[i][1] = ((float) rand()/RAND_MAX) * 5.0f - 2.0f;
   vel[i][2] = ((float) rand()/RAND_MAX) * 5.0f - 2.0f;

   acc[i][0] = 0.0f;
   acc[i][1] = 0.0f;
   acc[i][2] = 0.0f;

   force[i][0] = 0.0f;
   force[i][1] = 0.0f;
   force[i][2] = 0.0f;

   color[i][0] = 0.0f;//Red
   color[i][1] = 0.6f;//Green
   color[i][2] = 0.9f;//Blue
   color[i][3] = 1.0f;//transparancey
   startTime[i] = ((float) rand()/RAND_MAX);
}

// updates the particle table velocities, accelerations, and positions
void sumForces(float time)
{
   // for this timestep, accumulate all the forces that
   // act on each particle
   float disX;
   float disY;
   float disZ;
   float angle;
   float r;
   float fg;

   for (int i=0; i<NUM_PARTICLES; i++) {

      if (startTime[i] < 0) {
         // particle is active

         // ZERO ALL FORCES
         force[i][0] = force[i][1] = force[i][2] = 0.0;

         // GRAVITY
         force[i][1] = GRAVITY*mass[i];
         //wind force
         if ( pos[i][1] > 0.4f) {
            if ((time > 0.1f && time < 0.15f) || (time > 0.2f && time < 0.25f) ||
                (time > 0.3f && time < 0.35f) || (time > 0.4f && time < 4.5f) ||
                (time > 0.5f && time < 0.55f) || (time > 0.6f && time < 0.65f) ||
                (time > 0.7f && time < 0.75f) || (time > 0.8f && time < 0.85f)){
               force[i][0] += -30 * WIND;
               force[i][1] += 0 * WIND;
               force[i][1] += 0 * WIND;
            }
            else if (!(time > 0.9)){
               //wind Force
               force[i][0] += 30 * WIND;
               force[i][1] += 0 * WIND;
               force[i][1] += 0 * WIND;
            }
         }
         if (pos[i][1] >0.1f) {

            disX = pos[i][0] - 0.0f ;
            disY = pos[i][1] - 1.5f ;
            disZ = pos[i][2] - 0.0f ;

            r = sqrt(disX*disX + disY*disY + disZ*disZ);

            float cosxa = acos(disX/r);
            float cosya = acos(disY/r);
            float cosza = acos(disZ/r);

            //Move towards <0,2,0>
            force[i][0] += cos(cosxa) * -70 ;//5*(pos[i][0] /r);
            force[i][1] += cos(cosya) * -70 ;//5*(pos[i][1] /r);
            force[i][2] += cos(cosza) * -70 ;//5*(pos[i][2] /r);
         }
         // drag sphere drag = .47
         force[i][0] += -DRAG * pow(vel[i][0],2);
         force[i][1] += -DRAG * pow(vel[i][1],2);
         force[i][2] += -DRAG * pow(vel[i][2],2);
      }
   }
}

void eulerIntegrate() {
   // for each particle, compute the new velocity
   //   and position

   for (int i = 0; i < NUM_PARTICLES; i++) {
      if (startTime[i] < 0) {
         // particle is active
         // CALCULATE NEW ACCEL
         acc[i][0] = force[i][0] / mass[i];
         acc[i][1] = force[i][1] / mass[i];
         acc[i][2] = force[i][2] / mass[i];

         // CALCULATE NEW POS
         pos[i][0] += vel[i][0] * TIMESTEP +
            0.5 * acc[i][0] * TIMESTEP * TIMESTEP;
         pos[i][1] += vel[i][1] * TIMESTEP +
            0.5 * acc[i][1] * TIMESTEP * TIMESTEP;
         pos[i][2] += vel[i][2] * TIMESTEP +
            0.5 * acc[i][2] * TIMESTEP * TIMESTEP;


         // CALCULATE NEW VEL
         vel[i][0] += acc[i][0] * TIMESTEP;
         vel[i][1] += acc[i][1] * TIMESTEP;
         vel[i][2] += acc[i][2] * TIMESTEP;


         if (pos[i][1] >  2 || (pos[i][0] < -1.0f || pos[i][0] > 1.0f) ||
               (pos[i][2] < -1.0f || pos[i][2] > 1.0f)) {
            startTime[i] = -1;
            acc[i][0] = 0;
            acc[i][1] = 0;
            acc[i][2] = 0;
            pos[i][0] = 0;
            pos[i][1] = 0;
            pos[i][2] = 0;
            vel[i][0] = ((float) rand()/RAND_MAX) * 5.0f - 2.0f;
            vel[i][1] = ((float) rand()/RAND_MAX) * 5.0f - 2.0f;
            vel[i][2] = ((float) rand()/RAND_MAX) * 5.0f - 2.0f;
         }
      }
   }
}
// updates the particle table velocities, accelerations, and positions

//Called after the window and OpenGL are initialized. Called exactly once, before the main loop.
void init()
{
   glutMouseFunc(mouseCallback);
   glutMotionFunc(mouseMotion);
   InitializeProgram();

   for (int i=0; i<NUM_PARTICLES; i++) initializeParticleData(i);

   glGenVertexArrays(1, &vao);
   glBindVertexArray(vao);

   glEnable(GL_CULL_FACE);
   glCullFace(GL_BACK);
   glFrontFace(GL_CW);

   glEnable(GL_DEPTH_TEST);
   glDepthMask(GL_TRUE);
   glDepthFunc(GL_LEQUAL);
   glDepthRange(0.0f, 1.0f);
   glEnable(GL_DEPTH_CLAMP);
}

void drawStartSphere()
{
   glm::mat4 modelMatrix;
   Sphere *sphere3 = new Sphere();
   glUniformMatrix4fv(Points.worldSpaceMoveMatrixUnif, 1, GL_FALSE, glm::value_ptr(modelMatrix));

   modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, 0.0f, 0.0f));
   modelMatrix = glm::scale(modelMatrix, glm::vec3(0.01f, 0.01f, 0.01f));
   glUniformMatrix4fv(Points.modelToWorldMatrixUnif, 1, GL_FALSE, glm::value_ptr(modelMatrix));
   glUniform4f(Points.diffuseColorUnif, 1.0f, 1.0f, 1.0f, 1.0f);

   sphere3->DrawUnitSphere();
}

void changeColor(int i) {

   // Blue 0,.6,.9 //orange .9,.6,0
   float t ;
   float t2 ;
   float t3;
   float t4;
   if (pos[i][1] < 0.1f){
      t = (pos[i][1]- 0.0f)/(0.1 - 0.0f );

      color[i][0] = 0.0f + t * (0.9f - 0.0f) ;
      color[i][1] = 0.6f + t * (0.6f - 0.6f) ;
      color[i][2] = 0.9f + t * (0.0f - 0.9f) ;
   }
   if (pos[i][1] > 1.8f) {
      t2 = (pos[i][1]- 1.8f)/(2.0f - 1.8f );

      color[i][0] = 0.9f + t2 * (0.9f - 0.9f) ;
      color[i][1] = 0.6f + t2 * (0.3f - 0.6f) ;
      color[i][2] = 0.0f + t2 * (0.0f - 0.0f) ;

   }
   if (pos[i][0] > 0.2f ){
      t3 = (pos[i][0] - 0.2f)/(0.5f - 0.2f);
      color[i][0] = 0.9f + t3 * (0.9f - 0.9f) ;
      color[i][1] = 0.6f + t3 * (0.3f - 0.6f) ;
      color[i][2] = 0.0f + t3 * (0.0f - 0.0f) ;
   }
   if (pos[i][0] < -0.1f ){
      t4 = (pos[i][0] - -0.1f)/(-0.5f - -0.1f);
      color[i][0] = 0.9f + t4 * (0.9f - 0.9f) ;
      color[i][1] = 0.6f + t4 * (0.3f - 0.6f) ;
      color[i][2] = 0.0f + t4 * (0.0f - 0.0f) ;
   }
}

// draws all the particles
void drawParticles()
{
   // particles are just 8-sided spheres with no shading
   for (int i=0; i<NUM_PARTICLES; i++) {
      Sphere *sphere = new Sphere();

      glm::mat4 modelMatrix = glm::mat4();
      // move particle to its position
      modelMatrix = glm::translate(modelMatrix, glm::vec3(pos[i][0], pos[i][1], pos[i][2]));
      // make it tiny
      modelMatrix = glm::scale(modelMatrix, glm::vec3(0.01f, 0.01f, 0.01f));
      glUniformMatrix4fv(Points.modelToWorldMatrixUnif, 1, GL_FALSE, glm::value_ptr(modelMatrix));
      // particle's color
      changeColor(i);
      glUniform4f(Points.diffuseColorUnif, color[i][0],color[i][1], color[i][2], color[i][3]);

      sphere->DrawUnitSphere();
      //free(sphere);
   }
}

//Called to update the display.
//You should call glutSwapBuffers after all of your rendering to display what you rendered.
//If you need continuous updates of the screen, call glutPostRedisplay() at the end of the function.
void display()
{
   const float epsilon = 0.001f; // necessary for the trackball viewer
   const float period = 25.0;  // repeat time in seconds of movement
   float fElapsedTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f; // time since programs started in seconds
   float timeParameter = (float) fmodf(fElapsedTime, period)/period; // parameterized time


   // printf("time %f\n ", timeParameter);

   if(timeParameter >= .3 && timeParameter <= 0.9f && explode) {
      int r = rand() % 1000;
      printf("R is %d\n ", r);

      for (int i = 0; i < NUM_PARTICLES; i++) {
         acc[i][0] = acc[r][0];
         acc[i][1] = acc[r][1];
         acc[i][2] = acc[r][2];
         pos[i][0] = pos[r][0];
         pos[i][1] = pos[r][1];
         pos[i][2] = pos[r][2];
         vel[i][0] = ((float) rand()/RAND_MAX) * -10.0f + 5;
         vel[i][1] = ((float) rand()/RAND_MAX) * -10.0f + 5;
         vel[i][2] = 0.0;
         startTime[i] = -1;
      }
      explode = false;
   }
   // activate new particles
   // active particles have a start time of -1 until they age out or go off
   // the screen, at which time they are give a new start time (parameter)
   for (int i=0; i<NUM_PARTICLES; i++) {
      if (startTime[i] > 0.0f && startTime[i] < timeParameter)
         startTime[i] = -1.0f;
   }

   // update particle table for this time step
   sumForces(timeParameter);
   eulerIntegrate();

   glClearColor(0.0f, 0.0f, 0.2f, 0.0f); // unshaded objects will show up on this almost-black background
   glClearDepth(1.0f);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   // compute trackball interface (world to camera) transform --------------------------------
   // don't change any of this (unless you can make it better!)
   glm::mat4 camMatrix;
   glm::mat4 cam2Matrix;
   camMatrix = glm::translate(camMatrix, glm::vec3(modelTrans[0], modelTrans[1], modelTrans[2]));
   // in the middle of a left-button drag
   if (trackballMove) {
      // check to make sure axis is not zero vector
      if (!(-epsilon < axis[0] && axis[0] < epsilon && -epsilon < axis[1] && axis[1] < epsilon && -epsilon < axis[2] && axis[2] < epsilon)) {

         cam2Matrix = glm::rotate(cam2Matrix, angle, glm::vec3(axis[0], axis[1], axis[2]));
         cam2Matrix = cam2Matrix * (glm::mat4(trackballXform[0], trackballXform[1], trackballXform[2], trackballXform[3],
                  trackballXform[4], trackballXform[5], trackballXform[6], trackballXform[7],
                  trackballXform[8], trackballXform[9], trackballXform[10], trackballXform[11],
                  trackballXform[12], trackballXform[13], trackballXform[14], trackballXform[15]));
         glm::mat4 tempM = cam2Matrix;
         // copy current transform back into trackball matrix
         for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++)
               trackballXform[i*4 + j] = tempM[i][j];
         }
      }
   }
   camMatrix = camMatrix * (glm::mat4(objectXformPtr[0], objectXformPtr[1], objectXformPtr[2], objectXformPtr[3],
            objectXformPtr[4], objectXformPtr[5], objectXformPtr[6], objectXformPtr[7],
            objectXformPtr[8], objectXformPtr[9], objectXformPtr[10], objectXformPtr[11],
            objectXformPtr[12], objectXformPtr[13], objectXformPtr[14], objectXformPtr[15]));
   // end of world to camera transform -----------------------------------------------

   // pass it on to the shaders
   glUniformMatrix4fv(Points.worldToCameraMatrixUnif, 1, GL_FALSE, glm::value_ptr(camMatrix));

   // now draw the particles
   drawParticles();
   drawStartSphere();

   glutSwapBuffers();
   glutPostRedisplay();
}

//Called whenever the window is resized. The new window size is given, in pixels.
//This is an opportunity to call glViewport or glScissor to keep up with the change in size.
void reshape (int w, int h)
{
   glm::mat4 persMatrix = glm::perspective(45.0f, (w / (float)h), nearClipPlane, farClipPlane);
   winWidth = w;
   winHeight = h;

   glUseProgram(Points.theProgram);
   glUniformMatrix4fv(Points.cameraToClipMatrixUnif, 1, GL_FALSE, glm::value_ptr(persMatrix));

   glViewport(0, 0, (GLsizei) w, (GLsizei) h);
   glutPostRedisplay();
}

// Trackball-like interface functions - no need to ever change any of this--------------------------------------------------
float lastPos[3] = {0.0, 0.0, 0.0};
int curx, cury;
int startX, startY;

void trackball_ptov(int x, int y, int width, int height, float v[3]) {
   float d, a;
   // project x, y onto a hemisphere centered within width, height
   v[0] = (2.0f*x - width) / width;
   v[1] = (height - 2.0f*y) / height;
   d = (float) sqrt(v[0]*v[0] + v[1]*v[1]);
   v[2] = (float) cos((3.14159f/2.0f) * ((d<1.0f)? d : 1.0f));
   a = 1.0f / (float) sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
   v[0] *= a;
   v[1] *= a;
   v[2] *= a;
}

void mouseMotion(int x, int y) {
   float curPos[3], dx, dy, dz;

   if (zoomState == false && shiftState == false) {

      trackball_ptov(x, y, winWidth, winHeight, curPos);

      dx = curPos[0] - lastPos[0];
      dy = curPos[1] - lastPos[1];
      dz = curPos[2] - lastPos[2];

      if (dx||dy||dz) {
         angle = 90.0f * sqrt(dx*dx + dy*dy + dz*dz);

         axis[0] = lastPos[1]*curPos[2] - lastPos[2]*curPos[1];
         axis[1] = lastPos[2]*curPos[0] - lastPos[0]*curPos[2];
         axis[2] = lastPos[0]*curPos[1] - lastPos[1]*curPos[0];

         lastPos[0] = curPos[0];
         lastPos[1] = curPos[1];
         lastPos[2] = curPos[2];
      }

   }
   else if (zoomState == true) {
      curPos[1] = (float) y;
      dy = curPos[1] - lastPos[1];

      if (dy) {
         modelTrans[2] += dy * 0.01f;
         lastPos[1] = curPos[1];
      }
   }
   else if (shiftState == true) {
      curPos[0] = (float) x;
      curPos[1] =(float)  y;
      dx = curPos[0] - lastPos[0];
      dy = curPos[1] - lastPos[1];

      if (dx) {
         modelTrans[0] += dx * 0.01f;
         lastPos[0] = curPos[0];
      }
      if (dy) {
         modelTrans[1] -= dy * 0.01f;
         lastPos[1] = curPos[1];
      }
   }
   glutPostRedisplay( );
}

void startMotion(long time, int button, int x, int y) {
   if (!trackballEnabled) return;

   trackingMouse = true;
   redrawContinue = false;
   startX = x; startY = y;
   curx = x; cury = y;
   trackball_ptov(x, y, winWidth, winHeight, lastPos);
   trackballMove = true;
}

void stopMotion(long time, int button, int x, int y) {
   if (!trackballEnabled) return;

   trackingMouse = false;

   if (startX != x || startY != y)
      redrawContinue = true;
   else {
      angle = 0.0f;
      redrawContinue = false;
      trackballMove = false;
   }
}

// Called when a mouse button is pressed or released
void mouseCallback(int button, int state, int x, int y) {

   switch (button) {
   case GLUT_LEFT_BUTTON:
      trackballXform = (float *)objectXform;
      break;
   case GLUT_RIGHT_BUTTON:
   case GLUT_MIDDLE_BUTTON:
      trackballXform = (float *)lightXform;
      break;
   }
   switch (state) {
   case GLUT_DOWN:
      if (button == GLUT_RIGHT_BUTTON) {
         zoomState = true;
         lastPos[1] = (float) y;
      }
      else if (button == GLUT_MIDDLE_BUTTON) {
         shiftState = true;
         lastPos[0] = (float) x;
         lastPos[1] = (float) y;
      }
      else startMotion(0, 1, x, y);
      break;
   case GLUT_UP:
      trackballXform = (float *)lightXform; // turns off mouse effects
      if (button == GLUT_RIGHT_BUTTON) {
         zoomState = false;
      }
      else if (button == GLUT_MIDDLE_BUTTON) {
         shiftState = false;
      }
      else stopMotion(0, 1, x, y);
      break;
   }
}

// end of trackball mouse functions--------------------------------------------------------------------------


//Called whenever a key on the keyboard was pressed.
//The key is given by the ''key'' parameter, which is in ASCII.
void keyboard(unsigned char key, int x, int y)
{
   switch (key) {
   case 27:
      return;
   case 'z': modelTrans[2] += 1.0f; break;
   case 'Z': modelTrans[2] -= 1.0f; break;
   case 'x': modelTrans[0] += 1.0f; break;
   case 'X': modelTrans[0] -= 1.0f; break;
   case 'y': modelTrans[1] += 1.0f; break;
   case 'Y': modelTrans[1] -= 1.0f; break;
   case 'h':
             lightXformPtr[0] = objectXformPtr[0] = lightXformPtr[5] = objectXformPtr[5] =
                lightXformPtr[10] = objectXformPtr[10] = lightXformPtr[15] = objectXformPtr[15] = 1.0f;
             lightXformPtr[1] = objectXformPtr[1] = lightXformPtr[2] = objectXformPtr[2] = lightXformPtr[3] = objectXformPtr[3] =
                lightXformPtr[4] = objectXformPtr[4] = lightXformPtr[6] = objectXformPtr[6] = lightXformPtr[7] = objectXformPtr[7] =
                lightXformPtr[8] = objectXformPtr[8] = lightXformPtr[9] = objectXformPtr[9] = lightXformPtr[11] = objectXformPtr[11] =
                lightXformPtr[12] = objectXformPtr[12] = lightXformPtr[13] = objectXformPtr[13] = lightXformPtr[14] = objectXformPtr[14] = 0.0;
             modelTrans[0] = modelTrans[1] = 0.0; modelTrans[2] = -10.0;
             axis[0] = axis[1] = axis[2] = 0.0;
             angle = 0;
             break;
   case 'd':
             break;
   }
   glutPostRedisplay();
}

int main(int argc, char** argv)
{
   glutInit( &argc, argv );
   glutInitWindowPosition( 20, 20 );
   glutInitWindowSize( 500, 500 );
   glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
   glutCreateWindow("Lab 2 - Interpolated Motion");

   //test the openGL version
   getGLversion();

   init();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutMainLoop();
   return 0;
}
