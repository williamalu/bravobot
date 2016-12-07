void setupIR() {
  pinMode(IR_PINS[0], INPUT);
  pinMode(IR_PINS[1], INPUT);
  pinMode(IR_PINS[2], INPUT);
  pinMode(IR_PINS[3], INPUT);
}

boolean is_IR_ok() {  
  IR_Val[0] = analogRead(IR_PINS[0]) / 4;
  IR_Val[1] = analogRead(IR_PINS[1]) / 4;

  if (IR_Val[0] > 160 || IR_Val[1] > 160 || IR_Val[0] < 130 || IR_Val[1] < 130) {
    IR_range_exceed_count++;
  }

  if (IR_range_exceed_count > 2) {
    return false;
    IR_range_exceed_count = 0;
    last_IR_range_exceed = millis();
  }

  if (millis() - last_IR_range_exceed >= 100) {
    last_IR_range_exceed = millis();
    IR_range_exceed_count = 0;
  }
  
  return true;
}
