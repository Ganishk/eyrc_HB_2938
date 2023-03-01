 #include <AccelStepper.h>

AccelStepper stepper1(1, 3, 2); 
AccelStepper stepper2(1, 5, 4);
AccelStepper stepper3(1, 6, 7);
int scalefac=300;

float dirvec(float x1, float y1, float w1, float x2,float y2,float w2){
 float x=x2-x1;
 float y=y2-y1;
 float w=w2-w1;
 speedcalc(x,y,w);
}

float speedcalc(float x,float y,float w){
  float inv_mat[3][3]={1,0,-1,-0.5,-0.866,-1,-0.5,0.866,-1};
  float tar_mat[3][1]={x,y,w};
  float wheel_mat[3][1];
  for(int i=0;i<3;i++)
    for(int j=0;j<1;j++){
      wheel_mat[i][j]=0;
       for(int k=0;k<3;k++)
        wheel_mat[i][j] +=inv_mat[i][k]*tar_mat[k][j];
      }
  int v1=wheel_mat[0][0]*scalefac;
  int v2=wheel_mat[1][0]*scalefac;
  int v3=-wheel_mat[2][0]*scalefac;
  set_speed(v1,v2,v3); 
 }
 int set_speed( int v1,int v2,int v3){
unsigned long prev_timer=millis();

 while(millis()-prev_timer<3000){
     
    stepper1.setSpeed(v1);
    stepper2.setSpeed(v2);
    stepper3.setSpeed(v3);
    
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
 }
void setup() {
  stepper1.setMaxSpeed(1000); 
  stepper1.setAcceleration(500); 
  stepper1.setCurrentPosition(0);
  
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper2.setCurrentPosition(0);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  stepper3.setCurrentPosition(0);
}

void loop() {
 dirvec(0,0,0,0,1,0);
 dirvec(0,1,0,0,0,0);
 dirvec(0,0,0,1,0,0);
 dirvec(1,0,0,0,0,0);
 
 }
