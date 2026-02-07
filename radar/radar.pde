/**
 * Mr. Beep-A-Lot Radar Visualization
 * 
 * Real-time radar display for Mr. Beep-A-Lot's environment scanning mode.
 * Receives angle and distance data from Arduino via Serial communication
 * and renders an air trafiic controller radar screen with sweep animation.
 * 
 * Visual Elements:
 * - Green concentric circles (distance rings)
 * - Green grid lines (angle markers every 30°)
 * - Red sweep line (current scanning angle)
 * - Red dots (detected objects)
 * - Text overlay (angle and distance readout)
 * 
 * Serial Protocol:
 * - Receives: "angle,distance\n" (CSV format)
 * - Example: "45,120\n" = 45° angle, 120cm distance
 * - Baud rate: 9600
 * 
 * @file radar.pde
 * @author Shayan Mazahir
 * @date of last edit: February 2026
 * @requires Processing 3.x or higher
 * @requires Mr. Beep-A-Lot Arduino code running in Radar Mode
 */

// ============================================================================
// IMPORTS
// ============================================================================
import processing.serial.*;       // Serial communication library

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
Serial myPort;                    // Serial port object for Arduino communication
String receivedData;              // Raw data string received from Arduino
float angle = 0;                  // Current radar sweep angle (degrees)
float distance = 0;               // Current distance measurement (cm)

// ============================================================================
// SETUP - RUNS ONCE AT STARTUP
// ============================================================================
void setup() {
  // Window configuration
  size(800, 800);                 // Create 800x800 pixel window
  
  // Serial port initialization
  // NOTE: Serial.list()[0] is the first available port
  // If Arduino not detected, change index (e.g., [1], [2]) or print Serial.list()
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil('\n');       // Trigger serialEvent() when newline received
  
  // Initial display setup
  background(0);                  // Black background (radar screen aesthetic)
  noStroke();                     // Disable default stroke
  fill(255);                      // White fill for text (default)
}

// ============================================================================
// DRAW LOOP - RUNS CONTINUOUSLY (60 FPS default)
// ============================================================================
void draw() {
  // -------------------------------------------------------------------------
  // SCREEN RESET
  // -------------------------------------------------------------------------
  background(0);                  // Clear screen with black (creates animation effect)
  translate(width / 2, height / 2); // Move origin to center of screen
  
  // -------------------------------------------------------------------------
  // RADAR DISPLAY CONFIGURATION
  // -------------------------------------------------------------------------
  fill(0, 255, 0);                // Green color for radar elements
  noFill();                       // Don't fill shapes (wireframe mode)
  stroke(0, 255, 0);              // Green stroke for radar lines
  
  // -------------------------------------------------------------------------
  // DISTANCE RINGS (Concentric Circles)
  // -------------------------------------------------------------------------
  // Draw 4 circles representing distance markers
  // Each circle = 100cm increment (100cm, 200cm, 300cm, 400cm radius)
  for (int r = 100; r <= 400; r += 100) {
    ellipse(0, 0, r * 2, r * 2);  // ellipse(x, y, width, height)
                                   // Multiply by 2 because radius vs diameter
  }
  
  // -------------------------------------------------------------------------
  // ANGLE GRID LINES
  // -------------------------------------------------------------------------
  // Draw 12 radial lines (every 30°) from center to edge
  // Creates the classic radar grid pattern
  for (int a = 0; a < 360; a += 30) {
    float x = cos(radians(a)) * 400;  // Calculate endpoint x coordinate
    float y = sin(radians(a)) * 400;  // Calculate endpoint y coordinate
    line(0, 0, x, y);                 // Draw line from center to edge
  }
  
  // -------------------------------------------------------------------------
  // RADAR SWEEP LINE (Red rotating line)
  // -------------------------------------------------------------------------
  // This represents the current scanning direction
  float sweepX = cos(radians(angle)) * 400;   // Endpoint x based on current angle
  float sweepY = sin(radians(angle)) * 400;   // Endpoint y based on current angle
  stroke(255, 0, 0);                          // Red color for sweep line
  line(0, 0, sweepX, sweepY);                 // Draw from center to current angle
  
  // -------------------------------------------------------------------------
  // OBJECT DETECTION DISPLAY (Red dots)
  // -------------------------------------------------------------------------
  // If object detected within range, plot it on radar
  if (distance > 0 && distance <= 200) {      // Only draw if valid reading
    // Map distance from cm to pixels (0-200cm → 0-400px)
    float x = cos(radians(angle)) * map(distance, 0, 200, 0, 400);
    float y = sin(radians(angle)) * map(distance, 0, 200, 0, 400);
    fill(255, 0, 0);                          // Red fill for object dot
    ellipse(x, y, 10, 10);                    // Draw 10px diameter dot
  }
  
  // -------------------------------------------------------------------------
  // DATA READOUT OVERLAY
  // -------------------------------------------------------------------------
  // Display current angle and distance in top-left corner
  fill(255);                                  // White text
  textSize(16);                               // 16pt font
  // Position text relative to screen corners (accounting for translated origin)
  text("Radar Angle: " + angle + "°", -width / 2 + 10, -height / 2 + 20);
  text("Distance: " + distance + " cm", -width / 2 + 10, -height / 2 + 40);
}

// ============================================================================
// SERIAL EVENT HANDLER - Triggered when newline received
// ============================================================================
/**
 * Called automatically when Arduino sends data ending with '\n'
 * Parses incoming CSV data and updates global angle/distance variables
 * 
 * Expected format: "angle,distance\n"
 * Example: "45,120\n" sets angle=45, distance=120
 */
void serialEvent(Serial myPort) {
  // Read incoming data until newline character
  receivedData = myPort.readStringUntil('\n');
  
  if (receivedData != null) {                 // Verify data was received
    receivedData = trim(receivedData);        // Remove whitespace and newline
    String[] values = split(receivedData, ','); // Split CSV into array
    
    // Validate data format (must have exactly 2 values)
    if (values.length == 2) {
      angle = float(values[0]);               // Parse angle from first value
      distance = float(values[1]);            // Parse distance from second value
    }
    // If invalid format, ignore data (prevents crashes from corrupted data)
  }
}
