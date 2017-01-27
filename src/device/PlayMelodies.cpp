#include <Arduino.h>
#include "../conf.h"
#include "PlayMelodies.h"

void speedRunMelody() { // time warning sound
  int tempo = 600; // tempo in beats per minute

  // set duration of a whole note
  int millis_per_whole_note = 4.0 * 60.0 * 1000.0 / (float)tempo;

  int note_list[] = {
    NOTE_D5, 0, NOTE_D5, NOTE_D5, 0, NOTE_DS5, 0, NOTE_DS5, NOTE_DS5, 0, NOTE_E5, 0, NOTE_E5, NOTE_E5, 0, NOTE_F5, 0, NOTE_F5
  };

  float note_duration[] = {
    .25, .25, .25, .25, .5, .25, .25, .25, .25, .5, .25, .25, .25, .25, .25, .25, .25, 1.75
  };

  // send notes to playing function
  for (int this_note = 0; this_note < 18; this_note++) {
    playNote( note_list[this_note], (int)((float)note_duration[this_note] * (float)millis_per_whole_note));
  }
}



void crashMelody() { // either death sounds or game over sound
  
  int tempo = 400; // tempo in beats per minute

  // set duration of a whole note
  int millis_per_whole_note = 4.0 * 60.0 * 1000.0 / (float)tempo;

  int note_list[] = {
    NOTE_B4, NOTE_F5, 0, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_E4, 0, NOTE_E4, NOTE_C4
  };

  float note_duration[] = {
    .25, .25, .25, .25, .3333, .3333, .3333, .25, .25, .25, .25, .25
  };

  // send notes to playing function
  for (int this_note = 0; this_note < 12 ; this_note++) {
    playNote( note_list[this_note], (int)((float)note_duration[this_note] * (float)millis_per_whole_note));
  }
}



void speedFinishMelody() { // flagpole fanfare
  
  int tempo = 300; // tempo in beats per minute

  // set duration of a whole note
  int millis_per_whole_note = 4.0 * 60.0 * 1000.0 / (float)tempo;

  int note_list[] = {
    NOTE_G3, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_E5,
    NOTE_GS3, NOTE_C4, NOTE_DS4, NOTE_GS4, NOTE_C5, NOTE_DS5, NOTE_GS5, NOTE_DS5,
    NOTE_AS3, NOTE_D4, NOTE_F4, NOTE_AS4, NOTE_D5, NOTE_F5, NOTE_AS5, NOTE_AS5, NOTE_AS5, NOTE_AS5, NOTE_C6
  };

  float note_duration[] = {
    0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.5, 0.5,
    0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.5, 0.5,
    0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.5, 0.16666, 0.16666, 0.16666, 1
  };

  // send notes to playing function
  for (int this_note = 0; this_note < 27 ; this_note++) {
    playNote( note_list[this_note], (int)((float)note_duration[this_note] * (float)millis_per_whole_note));
  }
}

void searchFinishMelody() { //


  int tempo = 300; // tempo in beats per minute

  // set duration of a whole note
  int millis_per_whole_note = 4.0 * 60.0 * 1000.0 / (float)tempo;

  int note_list[] = {
    NOTE_G3, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_E5,
    NOTE_GS3, NOTE_C4, NOTE_DS4, NOTE_GS4, NOTE_C5, NOTE_DS5, NOTE_GS5, NOTE_DS5,
    NOTE_AS3, NOTE_D4, NOTE_F4, NOTE_AS4, NOTE_D5, NOTE_F5, NOTE_AS5, NOTE_AS5, NOTE_AS5, NOTE_AS5, NOTE_C6
  };

  float note_duration[] = {
    0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.5, 0.5,
    0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.5, 0.5,
    0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.16666, 0.5, 0.16666, 0.16666, 0.16666, 1
  };

  // send notes to playing function
  for (int this_note = 0; this_note < 27 ; this_note++) {
    playNote( note_list[this_note], (int)((float)note_duration[this_note] * (float)millis_per_whole_note));
  }
}

void stopMelody() { // pause sound
  int tempo = 400; // tempo in beats per minute

  // set duration of a whole note
  int millis_per_whole_note = (4.0 * 60.0 * 1000.0 / (float)tempo);

  int note_list[] = {
    NOTE_E6, NOTE_C6, NOTE_E6, NOTE_C6
  };

  float note_duration[] = { 
    0.16666, 0.16666, 0.16666, 0.5 
  };

  // send notes to playing function
  for (int this_note = 0; this_note < 4; this_note++) {
    playNote( note_list[this_note], (int)((float)note_duration[this_note] * (float)millis_per_whole_note));
  }
}



void startMelody() {
  int tempo = 600; // tempo in beats per minute

  // set duration of a whole note
  int millis_per_whole_note = (4.0 * 60.0 * 1000.0 / (float)tempo);

  int note_list[] = {
    NOTE_E5, NOTE_E5, 0, NOTE_E5, 0, NOTE_C5, NOTE_E5, 0, NOTE_G5, 0, NOTE_G4
  };

  float note_duration[] = { 
    0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.75, 1
  };

  // send notes to playing function
  for (int this_note = 0; this_note < 11; this_note++) {
    playNote( note_list[this_note], (int)((float)note_duration[this_note] * (float)millis_per_whole_note));
  }
}


void playNote(int note_frequency, int note_duration) {

  if (note_frequency > 0) {
    int pause_between_notes = 9;
    tone( BUZZER_PIN , note_frequency);
    delay(note_duration - pause_between_notes); // duration in millis
    noTone(BUZZER_PIN);
    delay(pause_between_notes);

    
//    analogWriteFrequency(BUZZER_PIN, note_frequency);
//    analogWrite(BUZZER_PIN, 512);
//    delay(note_duration - pause_between_notes); // duration in millis
//    analogWrite(BUZZER_PIN, 0);
//    delay(pause_between_notes);
    
  }
  else {
    delay(note_duration);
  }

  analogWriteFrequency(MOTOR_LF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RF_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_LB_PWM_PIN, 46875);
  analogWriteFrequency(MOTOR_RB_PWM_PIN, 46875);
}

