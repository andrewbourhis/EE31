//Collision deliverables.
//Input: Pins 2,3,18,19,20
//Output: Flags set indicating bumpers activated. True indicates the bumper is pressed, and vice versa.
//Usage: Assume all bumpers are in unpressed state on start, and initializing to false.
int front_PIN = 2, front_right_PIN = 3, front_left_PIN = 18, back_left_PIN = 19, back_right_PIN = 20;
volatile boolean front = false, front_right = false, front_left = false, back_left = false, back_right = false;


void setup() {
  //declare bumper sensors
  pinMode(front_PIN, INPUT);
  pinMode(front_right_PIN, INPUT);
  pinMode(front_left_PIN, INPUT);
  pinMode(back_left_PIN, INPUT);
  pinMode(back_right_PIN, INPUT);

  //attach interrupts
  setupInterrupts();
}

void setupInterrupts() {
  
  //attach the interrupts
  //note that:
  //RISING == triggers when pin goes from low to high.
  //FALLING == triggers when pin goes from high to low.
  //CHANGE == any changes.
  attachInterrupt(digitalPinToInterrupt(front_PIN),front_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(front_right_PIN),front_right_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(front_left_PIN),front_left_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(back_left_PIN),back_left_pressed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(back_right_PIN),back_right_pressed, CHANGE);
}

void front_pressed() {
  front = !front;
}

void front_right_pressed() {
  front_right = !front_right;
}

void front_left_pressed() {
  front_left = !front_left;
}

void back_left_pressed() {
  back_left = !back_left;
}

void back_right_pressed() {
  back_right = !back_right;
}


void loop() {
  //your own code.

}


