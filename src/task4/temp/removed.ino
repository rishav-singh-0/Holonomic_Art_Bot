// Creating the motor force variables

float f_front = 0;
float f_left = 0;
float f_right = 0;


// local velocity variables 

float l_x = 0;
float l_y = 0;

// global velocities veriables

float g_x = 0;
float g_y =0;

// angular position will be same for both frame local and global frame

float w = 0;


void draw_L(){

  // This function is prefrablly made for making "L" shape.


  // Giving constant Y velocity to travel vertical:

  l_y = 500;
  l_x = 0;
  w = 0;

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(100);


  // Comming back

  l_y = -500;
  l_x = 0;
  w = 0;

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(100);

  // Going to left side in x direction

  l_y = 0;
  l_x = -500;
  w = 0;

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(10000);
}


void draw_triangle(){

  // Giving x and y velocities in order to travel in 60 degree 

  l_y = 500 * 0.866025; // its 500 * sin(60)
  l_x = 500 * 0.5;      // its 500 * cos(60)
  w = 0;

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(100);


  // Going 60 degree down wards

  l_y = -500 * 0.866025; // its -500 * sin(60);
  l_x = 500 * 0.5;       // its 500 * cos(60)
  w = 0;

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(100);

  // Going to left side in x direction

  l_y = 0;
  l_x = -500;
  w = 0;

  get_f_wheels(l_x, l_y, w);

  // Setting the speed

  set_run(f_front, f_left, f_right);

  delay(4000);

  // Re-setting the speed

  set_run(0, 0, 0);

  delay(10000);
}