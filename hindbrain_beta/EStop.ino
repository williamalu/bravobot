void setupEStop() {
  pinMode(PHYSICAL_ESTOP,INPUT);
}

boolean is_EStop_ok() {
  if (analogRead(PHYSICAL_ESTOP) > 1000) {
    return false;
  }
  return true;
}
