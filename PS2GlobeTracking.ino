/* 
Uses PS2MouseHandler library to read a PS2 optical mouse
Hardware is Arduino Nano ESP32, HP PS\2 optical mouse with PAW3402 sensor chip, and a globe of the earth
Moving the mouse along the surface of the globe translates mouse xy movements to lat long
Contains code from various sources because spherical trigonometry is over my head
Calibration is performed by rotating the globe to the 'home' position, then reboot
*/

#include <Arduino.h>
#include <math.h>
#include <PS2MouseHandler.h>

const byte MOUSE_DATA = 8;
const byte MOUSE_CLOCK = 9;
struct Point {
    double x;
    double y;
};
unsigned long globeTimer = millis();
unsigned long serialTimer = millis();
Point mousePoint = {0,0}; 
Point currentLatLong = {32.0,-117.0}; // home location
const Point ORIGIN = {0, 0}; // origin is always 0.0
PS2MouseHandler mouse(MOUSE_CLOCK, MOUSE_DATA, PS2_MOUSE_REMOTE);

const double GLOBE_R = 6.0; // globe raduis in inches
const double MOUSE_RES = 1000; // mouse resolution in pixels per inch.
const double GLOBE_SCALE = (MOUSE_RES * GLOBE_R); 
const unsigned long GLOBE_REFRESH = 50;

char Lat[12];
char Long[12];

void setup() {
  Serial.begin(115000);
  if(mouse.initialise() != 0){
    Serial.println("mouse error");
  };
  mouse.set_resolution(MOUSE_RES); //the PAW3402 can be set to 500, 800, or 1000
  // do other setup here
}

void loop() {
  globe(); //check for updates
  if (millis() - serialTimer > 1000) { //serial output once per second
    serialTimer = millis();
    Serial.print("Lattitude : ");
    Serial.println(Lat);
    Serial.print("Longitude : ");
    Serial.println(Long);
  }
  //put code here
}

void globe() {
  if (millis() - globeTimer > GLOBE_REFRESH) { //refresh only every x ms
    globeTimer = millis();
    mouse.get_data(); //get mouse movements
    mousePoint = {(double)mouse.x_movement(), ((double)mouse.y_movement() * -1.0)}; //invert y-axis
    currentLatLong = calculateDestination(currentLatLong.x, currentLatLong.y, getVector(mousePoint).x, getVector(mousePoint).y); //compute our new lat long
    dtostrf(currentLatLong.x, 6, 6, Lat); //convert to char array
    dtostrf(currentLatLong.y, 6, 6, Long);
  }

}

Point getVector(Point point2) { //get bearing and distance based on mouse movement
  return {toDegrees(calculateBearing(ORIGIN, point2)), calculateDistance(ORIGIN, point2)};
}

// Calculate destination point given a starting point, bearing, and distance
Point calculateDestination(double startLat, double startLon, double bearing2, double distance2) {
    double delta = distance2 / GLOBE_SCALE; // Angular distance in radians using the scaling factor
    double lat1 = toRadians(startLat);
    double lon1 = toRadians(startLon);
    double brng = toRadians(bearing2);
    double lat2 = asin(sin(lat1) * cos(delta) + cos(lat1) * sin(delta) * cos(brng));
    double lon2 = lon1 + atan2(sin(brng) * sin(delta) * cos(lat1), cos(delta) - sin(lat1) * sin(lat2));

    // Normalize longitude to be between -180 and 180 degrees
    lon2 = fmod(lon2 + 3 * PI, 2 * PI) - PI;
    return {toDegrees(lat2), toDegrees(lon2)};
}

// Convert degrees to radians
double toRadians(double degrees) {
    return degrees * PI / 180.0;
}

// Convert radians to degrees
double toDegrees(double radians) {
    return radians * 180.0 / PI;
}
  
// Calculate bearing between two points
double calculateBearing(Point oldP, Point newP) {
    double dx = newP.x - oldP.x;
    double dy = newP.y - oldP.y;
    double angle = atan2(dy, dx);
    return fmod(toRadians(90.0) - angle, 2 * PI); // Convert to standard bearing
}

// Calculate distance between two points
double calculateDistance(Point oldP, Point newP) {
    double dx = newP.x - oldP.x;
    double dy = newP.y - oldP.y;
    return sqrt(dx*dx + dy*dy);
}

