// A fun sketch to demonstrate the use of the Tone library.
// By Brett Hagman
// bhagman@roguerobotics.com
// www.roguerobotics.com

// To mix the output of the signals to output to a small speaker (i.e. 8 Ohms or higher),
// simply use 1K Ohm resistors from each output pin and tie them together at the speaker.
// Don't forget to connect the other side of the speaker to ground!

// You can get more RTTTL (RingTone Text Transfer Language) songs from
// http://code.google.com/p/rogue-code/wiki/ToneLibraryDocumentation

// modified by: Sylwia Grudowska
// date: 26.02.2016

#include <avr/pgmspace.h>

#define OCTAVE_OFFSET 0

#define isdigit(n) (n >= '0' && n <= '9')

#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951

int notes[] = { 0,
                NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4,
                NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5, NOTE_B5,
                NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6, NOTE_A6, NOTE_AS6, NOTE_B6,
                NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7
              };

const char bond[] PROGMEM = "Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6";
const char goodbad[] PROGMEM = "GoodBad:d=4,o=5,b=56:32p,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,d#,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,c#6,32a#,32d#6,32a#,32d#6,8a#.,16f#.,32f.,32d#.,c#,32a#,32d#6,32a#,32d#6,8a#.,16g#.,d#";
const char ateam[] PROGMEM = "A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#";
const char missionimp[] PROGMEM = "MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d";
const char macgyver[] PROGMEM = "MacGyver:d=4,o=6,b=150:8b4,8e5,8a5,8b5,a5,8e5,8b4,8p,8e5,8a5,8b5,8a5,8e5,b4,8p,8e5,8a5,8b5,a5,8e5,8b4,8p,8a5,8d,8c,8d,8c,8b5,8a5,8b4,8e5,8a5,8b5,a5,8e5,8b4,8p,8e5,8a5,8b5,8a5,8e5,b4,8p,8e5,8a5,8b5,a5,8e5,8b4,8p,8a5,8d,8c,8d,8c,8b5,8a5,b5,8p,2b5,8p,b5,8p,a5,d.,b5,8p,2b5,8p,8b5,8p,2a5,p,8c,8c,8c,8c,8c,8c,2b5,16p,8f#5,8a5,8p,2g5,8p,8c,8c,8p,b5,8a5,8b5,8a5,8g5,8p,e,2a5,16p,8c,8c,8p,2b5,8p,8f#5,8a5,8p,2g5,8p,8c,8c,8p,4b5,8a5,8b5,8a5,8g5,8p,4e,2a5,2b5,32p,8c,8b5,8a5,c,8b5,8a5,8d,8c,8b5,d,8c,8b5,e,8d,8e,f#,b5,g,8p,f#,f,b5,8g,8e,8b5,8f#,8d,8a5,8e,8c,8g5,8d,8b5,8g5,8c,8e5,8b5,8d5,8c,8b5,8a5,8g5,a#5,a5,8g,8g5,8d,8g5,8d#,8d#5,8a#5,8a5,8g5,8g4,8d5,8g4,8d#5,8g4,8a#4,8a4,8g4,8g4,8g4,8g4,8g4,8g4,8g4";
const char entertainer[] PROGMEM = "Entertainer:d=4,o=5,b=140:8d,8d#,8e,c6,8e,c6,8e,2c.6,8c6,8d6,8d#6,8e6,8c6,8d6,e6,8b,d6,2c6,p,8d,8d#,8e,c6,8e,c6,8e,2c.6,8p,8a,8g,8f#,8a,8c6,e6,8d6,8c6,8a,2d6";
const char starwars[] PROGMEM = "StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6";
const char mahna[] PROGMEM = "MahnaMahna:d=16,o=6,b=125:c#,c.,b5,8a#.5,8f.,4g#,a#,g.,4d#,8p,c#,c.,b5,8a#.5,8f.,g#.,8a#.,4g,8p,c#,c.,b5,8a#.5,8f.,4g#,f,g.,8d#.,f,g.,8d#.,f,8g,8d#.,f,8g,d#,8c,a#5,8d#.,8d#.,4d#,8d#.";


///////////////////////////////////////////////
const char* const songs[] PROGMEM = {bond, goodbad, ateam, missionimp, macgyver, entertainer, starwars, mahna};
// this is large enough for the largest string it must hold
static char buffer[720];
static int TONE_PIN;

static void play_rtttl(const char*);

inline void set_pin(byte pin) {
  TONE_PIN = pin;
}

// http://www.learncpp.com/cpp-tutorial/59-random-number-generation/
inline unsigned int PRNG() {
  // our initial starting seed is 5323
  static unsigned int seed = 5323;

  // Take the current seed and generate a new value from it
  // Due to our use of large constants and overflow, it would be
  // very hard for someone to predict what the next number is
  // going to be from the previous one.
  seed = (8253729 * seed + 2396403);
  // Take the seed and return a value between 0 and 32767
  return seed % 32768;
}

inline byte random_number(int val_start, int val_end) {
  unsigned int factor = PRNG();
  unsigned int nr = factor;
  // Serial.println(factor);
  // get random generated number
  // and find first instance of digit from the range
  while (nr >= val_end) {
    nr = factor % 10;
    factor /= 10;

    // if digits in number are grater than range number
    // ex. range 0-8 and random number is 99999
    if (factor < val_end) {
      return 0;
    }
  }
  // Serial.println(nr);
  return nr;
}

inline void play_random_melody() {
  byte nr = random_number(0, 8);
  strcpy_P(buffer, (char*)pgm_read_word(&(songs[nr])));
  play_rtttl(buffer);
}

///////////////////////////////////////////////

void play_rtttl(const char *p) {
  // Absolutely no error checking in here
  byte default_dur = 4;
  byte default_oct = 6;
  int bpm = 63;
  int num = 0;
  long wholenote = 0;
  long duration = 0;
  byte note = 0;
  byte scale = 0;

  // format: d=N,o=N,b=NNN:
  // find the start (skip name, etc)

  while (*p != ':') {
    p++;  // ignore name
  }
  p++;  // skip ':'
  // get default duration
  if (*p == 'd') {
    p++; p++;// skip "d="
    num = 0;
    while (isdigit(*p)) {
      num = (num * 10) + (*p++ - '0');
    }
    if (num > 0) {
      default_dur = num;  // skip comma
    }
    p++;
  }

  // get default octave
  if (*p == 'o') {
    p++; p++; // skip "o="
    num = *p++ - '0';
    if (num >= 3 && num <= 7) default_oct = num;
    p++;  // skip comma
  }

  // get BPM
  if (*p == 'b') {
    p++; p++; // skip "b="
    num = 0;
    while (isdigit(*p)) {
      num = (num * 10) + (*p++ - '0');
    }
    bpm = num;
    p++;  // skip colon
  }

  // BPM usually expresses the number of quarter notes per minute
  wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)

  // now begin note loop
  while (*p) {
    // first, get note duration, if available
    num = 0;
    while (isdigit(*p)) {
      num = (num * 10) + (*p++ - '0');
    }

    if (num) {
      duration = wholenote / num;
    } else {
      duration = wholenote / default_dur;  // we will need to check if we are a dotted note after
    }
    // now get the note
    note = 0;

    switch (*p) {
      case 'c':
        note = 1;
        break;
      case 'd':
        note = 3;
        break;
      case 'e':
        note = 5;
        break;
      case 'f':
        note = 6;
        break;
      case 'g':
        note = 8;
        break;
      case 'a':
        note = 10;
        break;
      case 'b':
        note = 12;
        break;
      case 'p':
      default:
        note = 0;
    }
    p++;

    // now, get optional '#' sharp
    if (*p == '#') {
      note++; p++;
    }
    // now, get optional '.' dotted note
    if (*p == '.') {
      duration += duration / 2;
      p++;
    }
    // now, get scale
    if (isdigit(*p)) {
      scale = *p - '0';
      p++;
    } else {
      scale = default_oct;
    }
    scale += OCTAVE_OFFSET;

    if (*p == ',') {
      p++;  // skip comma for next note (or we may be at the end)
    }
    // now play the note
    if (note) {
      tone(TONE_PIN, notes[(scale - 4) * 12 + note]);
      Serial.println(notes[(scale - 4) * 12 + note]);
      delay(duration);
      noTone(TONE_PIN);
    } else {
      delay(duration);
    }
  }
}


