#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>

#include "MapDrawer.h"
// #include <GLUT/glut.h>
// #include <OpenGL/gl.h>
// #include <OpenGL/glu.h>
// #include <GL/glui.h>
 
#define PI 3.14159265358979323846
#define TWOPI 2*3.14159265358979323846
#define PID2 3.14159265358979323846/2.0
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
 
// Define superellipsoid parameters
 
float rx = 1.0;   // x-radius
float ry = 1.0;   // y-radius
float rz = 1.0;   // z-radius
float n1 = 2.0;  // first exponent
float n2 = 2.0;  // second exponent
 
// Define resolution
int res = 30;
 
int   main_window;
GLint spin= 0;
GLint spin1=0;
GLint egg= 0;
GLUI_Spinner    *spi[11];
 
//Viewer options (GluLookAt)
 
float fovy = 60.0, aspect = 1.0, zNear = 1.0, zFar = 100.0;
 
//Mouse modifiers
 
float depth = -1;
float scale=2.5;
float psi=0, theta=0;
float downX, downY;
bool leftButton = false, middleButton = false, rightButton=false;
int  showAxis;
int  showLight=1,showLight1=1;
int  sqstripes=-1;
int  chir;
 
void display(void);
int Squaredegg(void);
 
void myGlutIdle( void )
{
    if ( glutGetWindow() != main_window )
        glutSetWindow(main_window);
 
    glutPostRedisplay();
}
/*
   ============ 3D math routines
*/
 
//------ Returns the cross product of 2 vectors
 
void CrossProduct (double M[], double N[], double CS[]) {
    CS[0]=M[1] * N[2] - M[2] * N[1];
    CS[1]=M[2] * N[0] - M[0] * N[2];
    CS[2]=M[0] * N[1] - M[1] * N[0];
}
 
//------ Returns the length of a vector
 
double GetVectorLength (double M[]) {
    return sqrt( M[0] * M[0] + M[1] * M[1] + M[2] * M[2] );
}
 
//------ Returns the sum of two vectors
 
void   AddVectors (double M[], double N[], double R[]){
    R[0]= M[0] + N[0];
    R[1]= M[1] + N[1];
    R[2]= M[2] + N[2];
}
 
//------ Returns the vector scaled by the last parameter
 
void ScaleVector (double M[], double a, double N[]) {
    N[0]= M[0] * a;
    N[1]= M[1] * a; 
    N[2]= M[2] * a;
}
 
//------ Returns a normalized vector (length = 1)
 
void NormalizeVector (double M[],double R[3]) {
    double norm = GetVectorLength(M);
    if (norm == 0) norm  =1.0;
    ScaleVector( M, 1./ norm, R );
}
 
//------ Returns the unit normal vector of a triangle specified by the three
// points P1, P2, and P3.
 
void FindUnitNormal (double P1[3],double P2[3],double P3[3],double N[3]){
    double D1[3],D2[3];
    double R[3];
    D1[0]=P1[0]-P2[0];
    D1[1]=P1[1]-P2[1];
    D1[2]=P1[2]-P2[2];
    D2[0]=P2[0]-P3[0];
    D2[1]=P2[1]-P3[1];
    D2[2]=P2[2]-P3[2];
    CrossProduct(D1,D2,R);
    NormalizeVector(R,N);
}
 
//------ Initialization routine
 
void init () {
 
    GLfloat  mat_ambient[]  = { 0.25,     0.20725,  0.20725,  0 };
    GLfloat  mat_diffuse[]  = { 1.0,        0.829,    0.829,    0 };
    GLfloat  mat_specular[] = { 0.296648, 0.296648, 0.296648, 0 };
    GLfloat  light_diffuse[]  = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat  light_ambient[]  = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat  light_specular[] = { 0.5, 0.5, 0.5, 1.0 };
 
    GLfloat  light0_position[] = { -20.0, 20.0, 0.0, 0 };
    GLfloat  light1_position[] = { 10.0, 50.0, 0.0, 0 };
 
    GLfloat  shininess=0.088 * 128;
 
    glClearColor( 0  , 0  , 0  , 0   );    // Black background
    glShadeModel(GL_SMOOTH);       // Smooth shading
    glEnable(GL_MULTISAMPLE);      // Enable multisample antialiasing
    glEnable(GL_DEPTH_TEST);       // Enable hidden surface removal
    glEnable(GL_LIGHTING);
 
    // set light 0
 
    glEnable(GL_LIGHT0);
    glLightfv( GL_LIGHT0, GL_POSITION, light0_position );
    glLightfv( GL_LIGHT0, GL_DIFFUSE,  light_diffuse );
    glLightfv( GL_LIGHT0, GL_AMBIENT,  light_ambient );
    glLightfv( GL_LIGHT0, GL_SPECULAR, light_specular );
 
    // set light 1
 
    glEnable(GL_LIGHT1);
    glLightfv( GL_LIGHT1, GL_POSITION, light1_position );
    glLightfv( GL_LIGHT1, GL_DIFFUSE,  light_diffuse );
    glLightfv( GL_LIGHT1, GL_AMBIENT,  light_ambient );
    glLightfv( GL_LIGHT1, GL_SPECULAR, light_specular );
 
    // set material
 
    glMaterialfv( GL_FRONT, GL_AMBIENT,  mat_ambient );
    glMaterialfv( GL_FRONT, GL_DIFFUSE,  mat_diffuse );
    glMaterialfv( GL_FRONT, GL_SPECULAR, light_specular );
    glMaterialf( GL_FRONT, GL_SHININESS, shininess );
 
    egg=Squaredegg();
}
 
void display () {
 
    // Clear window
 
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
 
    glLoadIdentity();
    gluLookAt( 2, 4, 10, 0, 0, 0, 0, 1, 0 );
    glPushMatrix();
    glScalef( scale, scale, scale );
 
    //Motion Options
 
    glTranslatef(0.0, 0.0, -depth);
    glRotatef(-theta, 1.0, 0.0, 0.0);
    glRotatef(psi, 0.0, 1.0, 0.0);
 
    if (showLight) {
        glEnable(GL_LIGHT0);
    }
    else
    {
        glDisable(GL_LIGHT0);
    }
 
    if (showLight1) {
        glEnable(GL_LIGHT1);
    }
    else
    {
        glDisable(GL_LIGHT1);
    }
 
    glCallList(egg);            // draw the squared eggs
 
    glPopMatrix();
    glutSwapBuffers();
}
 
 
void SuperEllipsoidCurve(double theta1,double theta2,double p1,double p2, double x[])
{
    double tmp;
    double ct1,ct2,st1,st2;
 
    ct1 = cos(theta1);
    ct2 = cos(theta2);
    st1 = sin(theta1);
    st2 = sin(theta2);
 
    tmp  = sign(ct1) * pow(fabs(ct1),p1);
    x[0] = tmp * sign(ct2) * pow(fabs(ct2),p2);
    x[1] = sign(st1) * pow(fabs(st1),p1);
    x[2] = tmp * sign(st2) * pow(fabs(st2),p2);
}
 
int Squaredegg(void)
{
    GLuint  egg;
 
    double x[3],x1[3],x2[3],NN[3];
    double theta3;
 
    // Draw superellipsoid
 
    egg   = glGenLists(1);
    glNewList( egg, GL_COMPILE );
 
    double dt = 0.01 * TWOPI / res;
    for (int j=0;j<res/2;j++) {
        double theta1 = j * TWOPI / (double)res - PID2;
        double theta2 = (j + 1) * TWOPI / (double)res - PID2;
        glBegin(GL_QUAD_STRIP);
 
        for (int i=0;i<=res;i++) {
            double r1= rand() / double(RAND_MAX);
            double r2= rand() / double(RAND_MAX);
            double r3= rand() / double(RAND_MAX);
 
            GLfloat  mat_ambient[]  = { r1 ,     r2,  r3,  0 };
            glMaterialfv( GL_FRONT, GL_AMBIENT,  mat_ambient );      
 
            if (i == 0 || i == res)
                theta3 = 0;
            else
                theta3 = i * TWOPI / res;
 
            SuperEllipsoidCurve(theta2,theta3,n1,n2, x);
            SuperEllipsoidCurve(theta2+dt,theta3,n1,n2, x1);
            SuperEllipsoidCurve(theta2,theta3+dt,n1,n2, x2);
            FindUnitNormal (x1,x,x2,NN);
            glNormal3d(NN[0],NN[1],NN[2]);
 
            glTexCoord2f(i/(double)res,2*(j+1)/(double)res);
            glVertex3f(rx*x[0],ry*x[1],rz*x[2]);
 
            SuperEllipsoidCurve(theta1,theta3,n1,n2, x);
            SuperEllipsoidCurve(theta1+dt,theta3,n1,n2, x1);
            SuperEllipsoidCurve(theta1,theta3+dt,n1,n2, x2);
            FindUnitNormal (x1,x,x2,NN);
            glNormal3d(NN[0],NN[1],NN[2]);
 
            glTexCoord2f(i/(double)res,2*j/(double)res);
            glVertex3f(rx*x[0],ry*x[1],rz*x[2]);
 
        }
        glEnd();
    }
 
    glEndList();
 
    return egg;
 
}
 
 
void reshape (int w, int h)
{
    if (h == 0 || w == 0) return;  
 
    glViewport( 0, 0, w, h );
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective( 60, h ? w / h : 0, 1, 20 );
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
 
//------ Routine for rotating the scene
 
void spinDisplay (){
 
    int TimeNow = glutGet(GLUT_ELAPSED_TIME);
    int WaitUntil=0;
 
    if ( TimeNow >= WaitUntil ) {
        spin += 1;
 
        if ( spin > 360 )spin = spin - 360; 
        glutPostRedisplay();
        WaitUntil = TimeNow + 1000 / 25;    // 25 frames/s
    }
}
 
/* Callbacks */
void mouseCallback(int button, int state, int x, int y)
{
    downX = x; downY = y;
    leftButton = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN));
    rightButton = ((button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN));
    middleButton = ((button == GLUT_MIDDLE_BUTTON) &&  (state == GLUT_DOWN));
}
 
void motionCallback(int x, int y)
{
    if (leftButton) //Rotate
    {
        psi += (x-downX)/4.0;
        theta += (downY-y)/4.0;
    }
    if (rightButton) //Scale
    {
        if (depth + (downY - y)/10.0 < zFar-10 && depth + (downY - y)/10.0 > zNear+3)
            depth += (downY - y)/10.0;
    }
    downX = x;
    downY = y;
 
    glutPostRedisplay();
}
 
void control_cb( int control )
{
    egg=Squaredegg();
}
 
 
 
int main (int argc, char **argv)
{
    //Initialize GLUT
    glutInit(&argc, argv);
    //double buffering used to avoid flickering problem in animation
    glutInitDisplayMode(
            GLUT_DOUBLE         // Double buffering
            | GLUT_RGB            // RGB color mode
            | GLUT_DEPTH          // Hidden surface removal
            | GLUT_MULTISAMPLE    // Multisample antialiasing
            );
 
    // window size
    glutInitWindowSize( 600, 600 );
    // create the window 
    main_window = glutCreateWindow("Egg shapes");
    init();
    //Assign  the function used in events
    glutDisplayFunc(display);
 
    glutReshapeFunc( reshape );
    // mouse motion
    glutMouseFunc(mouseCallback);
    glutMotionFunc(motionCallback);
 
    /****************************************/
 
    GLUI *glui = GLUI_Master.create_glui( "Command & Control",300,500 );
 
    // Define resolution
    int res = 30;
    glui->add_statictext("Parameters for aperture shape");
    spi[0]= new GLUI_Spinner( glui, "X-radius (rx):", &rx,200,control_cb);
    spi[0] ->set_float_limits( 0.1, 10. );
    spi[0]->set_speed(0.1);
    spi[1]= new GLUI_Spinner( glui, "Y-radius (ry):", &ry,200,control_cb);
    spi[1] ->set_float_limits( 0.1, 10. );
    spi[1]->set_speed(0.1);
    spi[2]= new GLUI_Spinner( glui, "Z-radius (rz):", &rz,200,control_cb);
    spi[2] ->set_float_limits( 0.1, 10. );
    spi[2]->set_speed(0.1);
    spi[3]= new GLUI_Spinner( glui, "First exponent (n1):", &n1,200,control_cb);
    spi[3] ->set_float_limits( 0.1, 10. );
    spi[3]->set_speed(0.1);
    spi[4]= new GLUI_Spinner( glui, "Second exponent (n2):", &n2,200,control_cb);
    spi[4] ->set_float_limits( 0.1, 10. );
    spi[4]->set_speed(0.05);
    spi[5]= new GLUI_Spinner( glui, "Resolution (res):", &res,200,control_cb);
    spi[5] ->set_float_limits( 30, 300. );
    spi[5]->set_speed(0.1);
    glui->add_separator();
 
    new GLUI_Checkbox( glui, "Show Lights 0     ", &showLight);
    new GLUI_Checkbox( glui, "Show Lights 1    ", &showLight1);
 
    glui->add_separator();
 
    (new GLUI_Spinner( glui, "Scaling:", &scale,206, control_cb )) ->set_float_limits( 0, 20. );
 
    glui->add_separator();
 
    glui->add_button( "Quit", 0,(GLUI_Update_CB)exit );
    glui->set_main_gfx_window( main_window );
 
    /* We register the idle callback with GLUI, *not* with GLUT */
    GLUI_Master.set_glutIdleFunc( myGlutIdle );    
    //Let start glut loop
    glutMainLoop();
    return 0;
}