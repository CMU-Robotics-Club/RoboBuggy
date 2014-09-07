/**
 * @file encoder.c
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek0
 */

static int input_pin;

static int count=0;
// 1 if was high, 0 if was low
static int state=0;

void encoder_init(int encoder_pin) {
  input_pin = encoder_pin;
  pinMode(input_pin, INPUT);
}

// Every X ms, publish. (or perish!!)
void encoder_publish() {
  if (last_time < (millis() - print_period)) {
    Serial.println(count);
    last_time = millis();
  }
}

int encoder_get_count() {
  return count;
}

// Lightweight, checks low-pri encoder loop
void encoder_loop(){
  if (state != digital_read(input_pin)){
    count++;
    state = !state;
  }
}
