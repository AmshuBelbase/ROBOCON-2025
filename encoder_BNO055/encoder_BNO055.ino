cc
void odometry(){
  sensors_event_t event;
  bno.getEvent(&event);
  oldTick1=currentTick1;
  oldTick2=currentTick2;

  currentTick1 = encoder1.read(); 
  currentTick2 = encoder2.read();

  deltaTick1=currentTick1-oldTick1;
  deltaTick2=currentTick2-oldTick2;

  oldTheta = virtual_global_Theta;
  globalTheta = (event.orientation.x);
  Serial.println(globalTheta);
  //volatile int ThetaRad= globalTheta * PI / 180 ;
  // deltaTheta = (globalTheta - oldTheta);
  if (deltaTheta > 180) deltaTheta -= 360; 
  if (deltaTheta < -180) deltaTheta += 360;
  virtual_global_Theta+=deltaTheta;
  Serial.println(virtual_global_Theta);
  
  deltaTheta = (virtual_global_Theta - oldTheta) * PI / 180 ; 
  volatile float ThetaRad= virtual_global_Theta * PI / 180 ;
  deltaX=(deltaTick1 * C) - (L * deltaTheta);
  deltaY=(deltaTick2 * C) - (B * deltaTheta);

  double cos_theta = cos(ThetaRad);
  double sin_theta = sin(ThetaRad);

  double delta_x_global = cos_theta * deltaX - sin_theta * deltaY;
  double delta_y_global = sin_theta * deltaX + cos_theta * deltaY;

  globalX += delta_x_global;
  globalY += delta_y_global;
}

