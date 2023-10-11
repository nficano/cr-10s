// Adapted from sgp4 library
// Predicts time of next pass and start azimuth for satellites
long Predict(int many) {
  passinfo overpass;                      //structure to store overpass info
  sat.initpredpoint( timeNow , 0.0 );     //finds the startpoint

  bool error;
  for (int i = 0; i < many ; i++) {
    error = sat.nextpass(&overpass, 20);    //search for the next overpass, if there are more than 20 maximums below the horizon it returns false
    delay(0);

    if ( error == 1) { //no error, prints overpass information
      nextpassEpoch = (overpass.jdstart - 2440587.5) * 86400; //2440587.5 is the julian day at 1/1/1970 0:00 UTC
      AZstart = overpass.azstart;
      invjday(overpass.jdstart , TimeZone , true , years, months, days, hours, minutes, seconds); // Convert Julian date to print in serial.
#ifdef DEBUG
      
        Serial.println("Next pass for: " + String(satnames[SAT]) + " In: " + String(nextpassEpoch - timeNow));
        Serial.println("Start: az=" + String(overpass.azstart) + "° " + String(hours) + ':' + String(minutes) + ':' + String(seconds));
      
#endif
    }
    else
    {
#ifdef DEBUG
      Serial.println("Prediction error");
#endif
      while (true);
    }
    delay(0);
  }
  return nextpassEpoch;
}
