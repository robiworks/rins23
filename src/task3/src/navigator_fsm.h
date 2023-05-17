// Main state of the Navigator FSM
enum class FSMMainState {
  EXPLORING, // Exploration phase
  SEARCHING, // Searching for robbers
  PARKING,   // Parking at prison
  END        // Terminal state
};

// Child states of the EXPLORING main state
enum class FSMExploringState {
  EXPLORING,          // Driving around
  APPROACHING_FACE,   // Approaching detected face
  AT_FACE,            // Approached detected face
  DIALOGUE,           // Dialogue in progress
  SAVE_DIALOGUE,      // Save informative dialogue
  APPROACHING_POSTER, // Approaching detected poster
  AT_POSTER,          // Approached detected poster
  POSTER_OCR,         // Poster recognition in progress
  SAVE_POSTER,        // Save poster information
  CYLINDER_DETECTED,  // Cylinder detected, say color, save color and position
  RING_DETECTED,      // Ring detected, save color and position
  FINISHED            // Whole polygon explored
};

// Child states of the SEARCHING main state
enum class FSMSearchingState {
  DRIVING,          // Driving to cylinder
  AT_CYLINDER,      // At cylinder, extending arm
  LOOKING,          // Looking at face on top of cylinder
  ROBBER_FOUND,     // Robber found, tell to enter car
  ROBBER_NOT_FOUND, // Robber not on this cylinder
};

// Child states of the PARKING main state
enum class FSMParkingState {
  DRIVING,      // Driving to ring location
  FINDING_SPOT, // Finding parking point, extend arm
  MANEUVERING,  // Parking into spot
  FINISHED      // End of parking maneuver
};
