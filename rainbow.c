#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

#define TIME_STEP 16

#define X_RAMP 1.20
#define Z_RAMP -1.20

#define X_PLATFORM 0.40
#define Z_PLATFORM -0.40


/* FUNCTION PROTOTYPES */

int detect_ball(const double *color); // look for the ball and rotate until the trajectory is identified and centered
int grabs_ball(const double *color); // heads for the ball and, as soon as it reaches it, catches it with the rotational arm
int move_to_platform(const double *color); // moves the robot towards the platform
int move_to_ramp(); // moves the robot towards the ramp
int release_ball(); // release the ball when it reaches the ramp
void avoid_obstacle(const double *color); // allows the robot to avoid obstacles while taking the balls and when trying to reach the platform

void enable_sensors(); // enable all robot sensors
char* recognition_color(const double* color); // passing a color in rgb format returns a string with the recognized color
void move_robot(const char* command); // by default it moves the robot forward. Commands: LEFT - RIGHT - STOP - GO BACK - GO STRAIGHT
void move_arm(const char* command); // opens and closes the rotational arm. Commands: OPEN - CLOSE
const WbCameraRecognitionObject* is_that_color(const double* color, const WbCameraRecognitionObject *objects, int n_objects); // search in the camera object list for the ball of the specified color and returns its position in the list
const double odometry(const double Xf, const double Zf); // it allows to calculate the distance between the start and end point and returns the angle of rotation with respect to the target
void goal_oriented(const double theta_primo); // directs the robot in the direction of the objective to be achieved
double distance_two_points(const double X0, const double Z0, const double Xf, const double Zf); // calculates the distance between the starting point and the ending point


/* GLOBAL VARIABLES */

WbDeviceTag camera;
WbDeviceTag wheels[4];
WbDeviceTag gps;
WbDeviceTag ds[2]; //left and right sensor
WbDeviceTag ds_ball; // sensor for catching the ball
WbDeviceTag rotating_arm;
WbDeviceTag inertial_unit;


/* MAIN FUNCTION */

int main(int argc, char **argv) {

  // primary colors of newton
  const double red[3] = {1, 0, 0};
  const double yellow[3] = {1, 1, 0};
  const double green[3] = {0, 1, 0};
  const double blue[3] = {0, 0, 1};
  const double violet[3] = {0.56, 0, 1};
  
  const double *colors[5] = {red, yellow, green, blue, violet};
 
  // necessary to initialize webots stuff
  wb_robot_init();
  
  // enable all robot sensors
  enable_sensors();
   
  
  int i;
  int state;

  // iterates the array of balls
  for(i = 0; i < 5; i++){
    state = 1; // we assign the initial state to the robot
    while(state != 0){
      
      // state management
      switch(state){
        
        case 1:
          state = detect_ball(colors[i]);
          break;
          
        case 2:
          state = grabs_ball(colors[i]);
          break;
          
        case 3:
          state = move_to_platform(colors[i]);
          break;
          
        case 4:
          state = release_ball();
          break;
          
        case 5:
          state = move_to_ramp();
          break;
         
        default:
          printf("Error state\n");
      
      }
    }
  }
  
 
  wb_robot_cleanup();

  return 0;
}


/* FUNCTION DEFINITIONS */

// look for the ball and rotate until the trajectory is identified and centered
int detect_ball(const double *color){

  printf("---------------------\nDETECT BALL\n---------------------\n");
  printf("Searching %s ball\n", recognition_color(color));
  
  int n_objects = wb_camera_recognition_get_number_of_objects(camera);
  int prev_n_objects = n_objects;
  const WbCameraRecognitionObject *ball_pointer;
  
  // search for the ball of the specified color
  do{
  
    move_robot("LEFT");
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    n_objects = wb_camera_recognition_get_number_of_objects(camera);
    ball_pointer = is_that_color(color, objects, n_objects);    
  }
  
  while( ball_pointer == NULL);
  printf("%s ball found!\n", recognition_color(color)); 
  move_robot("STOP");
  
  // center for the ball of the specified color
  do{
    move_robot("LEFT");
    
    n_objects = wb_camera_recognition_get_number_of_objects(camera);
    
    // if the number of objects in the camera changes, the pointer must be updated with the new position of the ball in the list
    if(n_objects != prev_n_objects){
      const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
      ball_pointer = is_that_color(color, objects, n_objects);
      prev_n_objects = n_objects;
    }
    
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    ball_pointer = is_that_color(color, objects, n_objects);
    
    printf("Position: %d\n", ball_pointer->position_on_image[0]);
  }
  
  while( !(ball_pointer->position_on_image[0] > 395 && ball_pointer->position_on_image[0] < 405) );
  
  move_robot("STOP");
  //wb_robot_step(TIME_STEP);
  printf("Centered\n");
  
  return 2; // brings the robot to the state grabs_ball


}


// search in the camera object list for the ball of the specified color and returns its position in the list
const WbCameraRecognitionObject* is_that_color(const double* color, const WbCameraRecognitionObject *objects, int n_objects){

  // variables declaration
  int i;
  
  // search for the ball of the specified color from the camera object list
  for(i = 0; i < n_objects; i++, objects++){
  
    if(color[0] == objects->colors[0] && color[1] == objects->colors[1] && color[2] == objects->colors[2]){        
      return objects; // returns the position of the ball in the camera object list
    }
  }
  
  return NULL; // color not found
  
}



// heads for the ball and, as soon as it reaches it, catches it with the rotational arm
int grabs_ball(const double *color){
  
  printf("---------------------\nGRABS BALL\n---------------------\n");
  printf("Go to the ball\n");
  
  // move the robot until it is close to the ball
  while(wb_distance_sensor_get_value(ds_ball) > 950){
    move_robot("GO STRAIGHT");
    avoid_obstacle(color);
  }  
  
  // close the rotational arm
  move_arm("CLOSE");
  
  // stop the robot
  move_robot("STOP");
  
  printf("Captured!\n");
  
  return 3; // brings the robot to the state move_to_platform
}

// moves the robot towards the platform
int move_to_platform(const double *color){

   // takes the coordinates of the robot
  double X0 = wb_gps_get_values(gps)[0];
  double Z0 = wb_gps_get_values(gps)[2];
  
  printf("---------------------\nMOVE TO PLATFORM\n---------------------\n");
  
  // calculate odometry and direct the robot to the platform
  const double theta_platform = odometry(X_PLATFORM, Z_PLATFORM);
  goal_oriented(theta_platform);
  printf("Orientated to platform\n");
  
  // it goes on until it reaches the platform
  while( distance_two_points(X0, Z0, X_PLATFORM, Z_PLATFORM) > 0.03 ) {
      move_robot("GO STRAIGHT");
      X0 = wb_gps_get_values(gps)[0];
      Z0 = wb_gps_get_values(gps)[2];
      printf("Remaining distance: %.3f\n", distance_two_points(X0, Z0, Z_PLATFORM, Z_PLATFORM));
      avoid_obstacle(color); // avoid obstacles on his way to the platform
  }
  
  return 5; // brings the robot to the state move_to_ramp
}

// moves the robot towards the ramp
int move_to_ramp(){

  // takes the coordinates of the robot
  double X0 = wb_gps_get_values(gps)[0];
  double Z0 = wb_gps_get_values(gps)[2];
  
  printf("---------------------\nMOVE TO RAMP\n---------------------\n");

  // calculate odometry and direct the robot to the ramp
  const double theta_ramp = odometry(X_RAMP, Z_RAMP);
  goal_oriented(theta_ramp);
  printf("Orientated to ramp\n");
  
  // it goes on until it reaches the ramp
  while( distance_two_points(X0, Z0, X_RAMP, Z_RAMP) > 0.03 ) {
      move_robot("GO STRAIGHT");
      X0 = wb_gps_get_values(gps)[0];
      Z0 = wb_gps_get_values(gps)[2];
      printf("Remaining distance: %.3f\n", distance_two_points(X0, Z0, X_RAMP, Z_RAMP));
  }
  
  return 4; // brings the robot to the state release
}

// it allows to calculate the distance between the start and end point and returns the angle of rotation with respect to the target
const double odometry(const double Xf, const double Zf){

  // takes the coordinates of the robot
  double X0 = wb_gps_get_values(gps)[0];
  double Z0 = wb_gps_get_values(gps)[2];

  // calculate the distance
  const double distance = distance_two_points(X0, Z0, Xf, Zf);
  printf("distance:%.2f\nXA: %f\nZA: %f\n", distance, X0, Z0);
  
  // calculates the angle of rotation with respect to the target
  const double delta_rot = atan2(Xf-X0, Zf-Z0);
  printf("delta_rot: %f\n", delta_rot);
  
  return delta_rot; // return the angle of rotation with respect to the target
}

// directs the robot in the direction of the objective to be achieved
void goal_oriented(const double theta_primo){
  
  // takes the yaw angle
  double theta = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit)[2];
  
  // calculate the difference between the two angles
  double diff = theta_primo - theta;
  
  printf("Diff: %f\n", diff);
  
  // depending on the angle, it turns right or left
  if(diff > 0){
    while(diff > 0.01){
    move_robot("LEFT");
    theta = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit)[2];
    diff = theta_primo - theta;
    }
  }
  
  else{
    while(diff < -0.01){
    move_robot("RIGHT");
    theta = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit)[2];
    diff = theta_primo - theta;
    }
  }
  
}

// calculates the distance between the starting point and the ending point
double distance_two_points(const double X0, const double Z0, const double Xf, const double Zf){
  return sqrt( pow(Xf - X0, 2) + pow(Zf - Z0, 2) );
}

// allows the robot to avoid obstacles while taking the balls and when trying to reach the platform
void avoid_obstacle(const double *color){

  // variables declaration
  int obstacle_counter = 140;
  int turn_right = 280;
  bool right_obstacle;
  bool left_obstacle;
  double ds_values[2];
  double ds_ball_value;
  bool ball_in;
  
  //read sensors
  for(int i = 0; i < 2; i++ ){
      ds_values[i] = wb_distance_sensor_get_value( ds[i] );
      left_obstacle = ds_values[0] < 900;
      right_obstacle = ds_values[1] < 900;
  }
  
  
  ds_ball_value = wb_distance_sensor_get_value( ds_ball );
  ball_in = ds_ball_value < 950; // true if the ball is in the arms
    
  //avoid obstacle  
  while( obstacle_counter != 0 ){   
    obstacle_counter--;
    
    // go around the obstacle to the right if the obstacle is present
    if (left_obstacle) {
      printf("Obstacle detect \n");
      while(obstacle_counter != 0){
        move_robot("RIGHT");
        obstacle_counter--;       
      }
      
      for(int i = 0; i<120; i++){
        move_robot("GO STRAIGHT");
      }
      move_robot("STOP");
      
      // if the ball is inside the arms it goes towards the platform
      if(ball_in){
        move_to_platform(color);
      }
      // if the ball is not inside the arms, look for it
      else {
        detect_ball(color);
      }
    }
    // go around the obstacle to the left if the obstacle is present
    else if (right_obstacle) {
      printf("Obstacle detect \n");
      while(obstacle_counter != 0){
        move_robot("LEFT");
        obstacle_counter--;     
      }
      
      for(int i = 0; i<120; i++){
        move_robot("GO STRAIGHT");
      }
      move_robot("STOP");
      
      while(turn_right != 0){
        move_robot("RIGHT");
        turn_right--;
      }
      
      // if the ball is inside the arms it goes towards the platform
      if(ball_in){
        move_to_platform(color);
      }
      // if the ball is not inside the arms, look for it
      else {
        detect_ball(color);
      }
    }
    // if there is no obstacle
    else{
      obstacle_counter = 0;
    } 
  }
}

// release the ball when it reaches the ramp
int release_ball(){

  move_arm("OPEN");
  move_robot("STOP");
  
  int i;
  for( i = 0; i < 550; i++){
    move_robot("GO BACK");
  }
  
  move_robot("STOP");
  
  return 0; // exits the while in the state management
}



// enable all robot sensors
void enable_sensors(){

  // variables declaration
  int i;

  // enable GPS
  gps = wb_robot_get_device("global");
  wb_gps_enable(gps, TIME_STEP);
  
  // enable inertial unit
  inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  
  // enable sensor to catch the ball
  ds_ball = wb_robot_get_device("ds_ball");
  wb_distance_sensor_enable(ds_ball, TIME_STEP);
  
  // enable sensors for detect an obstacle
  char ds_name[2][10] = { "ds_left", "ds_right" };
  
  for( i = 0; i < 2; ++i ){
    ds[i] = wb_robot_get_device( ds_name[i] );
    wb_distance_sensor_enable( ds[i], TIME_STEP );
  }
  
  // enable the rotating arm
  rotating_arm = wb_robot_get_device("rm");
  
  // enable the camera
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  
  // enable the four wheels
  char wheels_name[4] [8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  
  for(i=0; i<4; i++){
    wheels[i] = wb_robot_get_device(wheels_name[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
}

// passing a color in rgb format returns a string with the recognized color
char* recognition_color(const double* color){
  
  if(color[0] == 1 && color[1] == 0 && color[2] == 0){
    return "RED";
  }
  
  if(color[0] == 1 && color[1] == 1 && color[2] == 0){
    return "YELLOW";
  }
  
  if(color[0] == 0 && color[1] == 1 && color[2] == 0){
    return "GREEN";
  }
  
  if(color[0] == 0 && color[1] == 0 && color[2] == 1){
    return "BLUE";
  }
  
  if(color[0] == 0.56 && color[1] == 0 && color[2] == 1){
    return "VIOLET";
  }
  
  return "UNKNOWN";
}


// by default it moves the robot forward. Commands: LEFT - RIGHT - STOP - GO BACK - GO STRAIGHT
void move_robot(const char* command){
  
  // variables declaration. By default the robot moves forward
  double left_wheels_speed = 0.5;
  double right_wheels_speed = 0.5;
  
  // turn left
  if(strcmp(command, "LEFT") == 0) {
    left_wheels_speed = -0.5;
  }
  
  // turn right
  else if(strcmp(command, "RIGHT") == 0){
    right_wheels_speed = -0.5;
  }
  
  // stop
  else if(strcmp(command, "STOP") == 0){
    left_wheels_speed = 0.0;
    right_wheels_speed = 0.0;
  }
  
  // go straight
  else if(strcmp(command, "GO STRAIGHT") == 0){
    left_wheels_speed = 3.0;
    right_wheels_speed = 3.0;
  }
  
  // go back
  else if(strcmp(command, "GO BACK") == 0){
    left_wheels_speed = -3.0;
    right_wheels_speed = -3.0;
  }
  
  else{
    printf("Command not found!\n");
  }
  
  // set the speed on the wheels
  wb_motor_set_velocity(wheels[0], left_wheels_speed);
  wb_motor_set_velocity(wheels[1], right_wheels_speed);
  wb_motor_set_velocity(wheels[2], left_wheels_speed);
  wb_motor_set_velocity(wheels[3], right_wheels_speed);
  
  wb_robot_step(TIME_STEP);
}


// opens and closes the rotational arm. Commands: OPEN - CLOSE
void move_arm(const char* command){

  // variables declaration
  double rotate;

  // open rotational arm
  if(strcmp(command, "OPEN") == 0) {
    rotate = 0;
  }
  
  // close rotational arm
  if(strcmp(command, "CLOSE") == 0) {
    rotate = -1.57;
    
  }
  
  // set the position to rotational arm
  wb_motor_set_position(rotating_arm, rotate);
  wb_robot_step(TIME_STEP);
  
}