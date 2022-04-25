#pragma once

int farthest_value=1000;
int farthest_position = 0;

int farthest() {
  // Take the radar array and returns the index that is the farthest away.

  farthest_value=1000;
  farthest_position = 0;
  
  for(int i = 0; i < 5; i++) {
    if(radar[i] < farthest_value) {
      farthest_value = radar[i];
      farthest_position = i;
    }
  }
  return farthest_position;
}
