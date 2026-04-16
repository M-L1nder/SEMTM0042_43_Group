// Adjustments made by DV22277 - Michael Linder:
// Adjusting calibration constants for wheel radius and seperation over varius iterations to achieve best results in foraging challenge
// Minor change to kinematics to use average angle to calculate x,y position to try and improve accuracy as my robot never exactly spun on the spot
// So the purely rotate on spot and move in straight line assumption breaks down over 4 minutes/repetitive cycling in straight lines

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

// These two commands mean that this header file
// will attempt to use some global variables of
// the same name found in another header file.
// From encoders.h
extern volatile long count_e0;
extern volatile long count_e1;

// Some global definitions concerning
// the robot dimensions.  You will need
// to calibrate these to get the best
// performance. (see Labsheet 4)
const float count_per_rev = 358.3;   // From documentation - correct from testing
// pushing in a straight line and reading counts from motors led interesting results
// Some times would have left counts lower than right others viceversa - no predictble pattern.
// Usually this would be combatted with effective different radii on the wheels but no combination
// Seemed to improve results in testing so reverted back to single wheel radius and counts

// Wheel radius in measurement is 16mm and seperation ~ 85mm wheel to wheel these two calibrations match approximately
const float wheel_radius  = 15.0 * (111.76 / 101.76) * (100.0 / 103.85) * (105 / 102.78); //Comes to 16.2mm - ORIGINAL Value 15.0.
const float wheel_sep = 42.1; //42.607;  // mm, from centre of robot to wheel centre

// Notes to self when going around all points - without and with the calibration turns for the sensors - as robot turns anticlockwise and clockwise
// Overall displacement can be narrowed down to under and overturning as one direction of turn is dominant
// Notes on wheel seperation:
// too small - end up down and right of the start - Underturning
// too big   - end up  up and left  - Overturning
// Examples:
// 42.45 down and right after 2 laps
// 42.5  visibly closer after 2 laps but down and right 42.55 roughly same
// 42.65 similar but closer
// 42.8 Up and left after two laps so around 42.65 is sweet spot

// Aftering including the calibration spins, the robot's internal drift is exacerbated
// Needed to slightly tweak after the spins were added as minor issues with heading were amplified after spinning on the spot
// Iteration used to narrow down to 42.61mm through trying larger and smaller seperations - halfving the difference each iteration to narrow search
// Examples:
// 42 -> left of point 1
// 43 -> right of point 1
// 42.7 undershooting 2,3,4
// 42.65 similar
// 42.2 under shooting position 1 and overshooting 2,3,4


// Take the circumference of the wheel and divide by the
// number of counts per revolution. This provides the mm
// travelled per encoder count.
const float mm_per_count  = ( 2.0 * wheel_radius * PI ) / count_per_rev;

// Class to track robot position.
class Kinematics_c {
  public:

    // Pose
    float x, y, theta;

    // To calculate the difference
    // in encoder counts for each
    // call to update()
    long last_e1;
    long last_e0;

    // Constructor, must exist.
    Kinematics_c() {

    }

    // Used to setup kinematics, and to set a start position
    void initialise( float start_x, float start_y, float start_th ) {
      last_e0 = count_e0; // Initisalise last count to current count
      last_e1 = count_e1; // Initisalise last count to current count
      x = start_x;
      y = start_y;
      theta = start_th;
    }


    // Here I have opted to use encoder counts rather than
    // wheel velocity.  Either way will work.
    // With velocity, the difference in time between updates
    // is required (distance = speed / time ).
    // If we use the velocity, it means we have to do
    // extra computation just to get back to distance, which
    // we had in the first place (as change of encoder counts)
    void update( ) {

      long delta_e1;  // change in counts
      long delta_e0;  // change in counts
      float mean_delta;

      float x_contribution;   // linear translation
      float th_contribution;  // rotation

      // How many counts since last update()?
      delta_e1 = count_e1 - last_e1;
      delta_e0 = count_e0 - last_e0;

      // Used last encoder values, so now update to
      // current for next iteration
      last_e1 = count_e1;
      last_e0 = count_e0;

      // Work out x contribution in local frame.
      mean_delta = (float)delta_e1;
      mean_delta += (float)delta_e0;
      mean_delta /= 2.0;

      x_contribution = mean_delta * mm_per_count;

      // Work out rotation in local frame
      th_contribution = (float)delta_e0;
      th_contribution -= (float)delta_e1;
      th_contribution *= mm_per_count;
      th_contribution /= (wheel_sep * 2.0);

      // Update global frame by taking the midpoint of theta contribution
      // This approximates a very small curve and almost filters the theta value 
      // like using alpha for speed calcs - this uses half of the 'new' and half 'old'
      // Slightly moves away from the pure rotation then straight, 
      // as my robot never span exactly on the spot
      // Unsure if correct but helped with waypoint accuracy and tracking position during spin test
      
      float avg_theta = theta + (th_contribution / 2.0);
      x = x + x_contribution * cos(avg_theta);
      y = y + x_contribution * sin(avg_theta);
      theta += th_contribution;

      // Paul O'Dowd Code: 
      //        // Update global frame by taking these
      //        // local contributions, projecting a point
      //        // and adding to global pose.
      //        x = x + x_contribution * cos( theta );
      //        y = y + x_contribution * sin( theta );
      //        theta = theta + th_contribution;

      // Wrap angle of theta to ensure that there is no 'long way round' during travel 
      // Happened when comanding a square after doing 2 iterations, internal theta was >360 
      // so thought it needed to undo a full turn and then turn the extra 90 deg
      if (theta > PI)  theta -= 2.0 * PI;
      else if (theta <= -PI) theta += 2.0 * PI;

      // Done!
    } // End of update()

}; // End of Kinematics_c class defintion



#endif
