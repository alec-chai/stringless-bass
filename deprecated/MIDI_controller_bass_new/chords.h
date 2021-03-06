
#define NOTE_E2   82.41
#define NOTE_F2   87.31
#define NOTE_Fs2  92.50
#define NOTE_G2   98.00
#define NOTE_Gs2 103.82
#define NOTE_A2  110.00
#define NOTE_As2 116.54
#define NOTE_B2  123.47
#define NOTE_C3  130.81
#define NOTE_Cs3 138.59
#define NOTE_D3  146.83
#define NOTE_Ds3 155.56
#define NOTE_E3  164.81
#define NOTE_F3  174.61
#define NOTE_Fs3 185.00
#define NOTE_G3  196.00
#define NOTE_Gs3 207.65
#define NOTE_A3  220.00
#define NOTE_As3 233.08
#define NOTE_B3  246.94
#define NOTE_C4  261.63
#define NOTE_Cs4 277.18
#define NOTE_D4  293.66
#define NOTE_Ds4 311.13
#define NOTE_E4  329.63
#define NOTE_F4  349.23
#define NOTE_Fs4 369.99
#define NOTE_G4  392.00
#define NOTE_Gs4 415.30
#define NOTE_A4  440.00
#define NOTE_As4 466.16
#define NOTE_B4  493.88

// The equation for note to frequency is:
// float freq = 440.0f * exp2f((float)(note - 69) * 0.0833333f);

// according to http://www.guitar-chords.org.uk/
// and http://www.8notes.com/guitar_chord_chart/c.asp
//
              // open =  NOTE_E2  NOTE_A2  NOTE_D3  NOTE_G3  NOTE_B3  NOTE_E4
const float Cmajor[6] = {      0, NOTE_C3, NOTE_E3, NOTE_G3, NOTE_C4, NOTE_E4};  // C - E - G
const float Csmajor[6] = {      0, NOTE_Cs3, NOTE_F3, NOTE_Gs3, NOTE_Cs4, NOTE_F4};  // C# - F - G#
const float Dmajor[6] = {      0,       0, NOTE_D3, NOTE_A3, NOTE_D4, NOTE_Fs4}; // D - F# - A
const float Dsmajor[6] = {      0,       0, NOTE_Ds3, NOTE_As3, NOTE_Ds4, NOTE_G4}; // D# - G - A#
const float Emajor[6] = {NOTE_E2, NOTE_B2, NOTE_E3, NOTE_Gs3,NOTE_B3, NOTE_E4};  // E - G# - B
const float Fmajor[6] = {      0, NOTE_A2, NOTE_F3, NOTE_A3, NOTE_C4, NOTE_F4};  // F - A - C
const float Fsmajor[6] = {      0, NOTE_As2, NOTE_Fs3, NOTE_As3, NOTE_Cs4, NOTE_Fs4};  // F# - A# - C#
const float Gmajor[6] = {NOTE_G2, NOTE_B2, NOTE_D3, NOTE_G3, NOTE_B3, NOTE_D4};  // G - B - D
const float Gsmajor[6] = {NOTE_Gs2, NOTE_C3, NOTE_Ds3, NOTE_Gs3, NOTE_C4, NOTE_Ds4};  // G# - C - D#
const float Amajor[6] = {      0, NOTE_A2, NOTE_E3, NOTE_A3, NOTE_Cs4,NOTE_E4};  // A - C# - E
const float Asmajor[6] = {      0, NOTE_As2, NOTE_F3, NOTE_As3, NOTE_D4,NOTE_F4};  // A# - D - F
const float Bmajor[6] = {      0, NOTE_B2, NOTE_Fs3,NOTE_B3, NOTE_Ds4,NOTE_Fs4}; // B - D# - F#

const float Cminor[6] = {      0, NOTE_C3, NOTE_G3, NOTE_C4, NOTE_Ds4,NOTE_G4};  // C - D# - G
const float Csminor[6] = {      0, NOTE_Cs3, NOTE_Gs3, NOTE_Cs4, NOTE_E4,NOTE_Gs4};  // C# - E - G#
const float Dminor[6] = {      0,       0, NOTE_D3, NOTE_A3, NOTE_D4, NOTE_F4};  // D - F - A
const float Dsminor[6] = {      0,       0, NOTE_Ds3, NOTE_As3, NOTE_Ds4, NOTE_Fs4};  // D# - F# - A#
const float Eminor[6] = {NOTE_E2, NOTE_B2, NOTE_E3, NOTE_G3, NOTE_B3, NOTE_E4};  // E - G - B
const float Fminor[6] = {NOTE_F2, NOTE_C3, NOTE_F3, NOTE_Gs3,NOTE_C4, NOTE_F4};  // F - G# - C
const float Fsminor[6] = {NOTE_Fs2, NOTE_Cs3, NOTE_Fs3, NOTE_A3,NOTE_Cs4, NOTE_Fs4};  // F# - A - C#
const float Gminor[6] = {NOTE_G2, NOTE_D3, NOTE_G3, NOTE_As3,NOTE_D3, NOTE_G4};  // G - A# - D
const float Gsminor[6] = {NOTE_Gs2, NOTE_Ds3, NOTE_Gs3, NOTE_B3,NOTE_Ds3, NOTE_Gs4};  // G# - B - D#
const float Aminor[6] = {      0, NOTE_A2, NOTE_E3, NOTE_A3, NOTE_C4, NOTE_E4};  // A - C - E
const float Asminor[6] = {      0, NOTE_As2, NOTE_F3, NOTE_As3, NOTE_Cs4, NOTE_F4};  // A# - C# - F
const float Bminor[6] = {      0, NOTE_B2, NOTE_Fs3,NOTE_B3, NOTE_D4, NOTE_Fs4}; // B - D - F#

const float C7major[6] = {      0, NOTE_C3, NOTE_E3, NOTE_G3, NOTE_As3, NOTE_C4};  // C - E - G - Bb
const float Cs7major[6] = {      0, NOTE_Cs3, NOTE_F3, NOTE_Gs3, NOTE_B3, NOTE_Cs4};  // C# - F - G# -B
const float D7major[6] = {      0,       0, NOTE_D3, NOTE_A3, NOTE_C4, NOTE_Fs4}; // D - F# - A - C
const float Ds7major[6] = {      0,       0, NOTE_Ds3, NOTE_As3, NOTE_Cs4, NOTE_G4}; // D# - G - A# -C#
const float E7major[6] = {NOTE_E2, NOTE_B2, NOTE_E3, NOTE_Gs3,NOTE_B3, NOTE_D4};  // E - G# - B - D
const float F7major[6] = {      0, NOTE_A2, NOTE_F3, NOTE_A3, NOTE_C4, NOTE_Ds4};  // F - A - C - Eb
const float Fs7major[6] = {      0, NOTE_As2, NOTE_Fs3, NOTE_As3, NOTE_Cs4, NOTE_E4};  // F# - A# - C# - E
const float G7major[6] = {NOTE_G2, NOTE_B2, NOTE_D3, NOTE_F3, NOTE_B3, NOTE_D4};  // G - B - D - F
const float Gs7major[6] = {NOTE_Gs2, NOTE_C3, NOTE_Ds3, NOTE_Fs3, NOTE_C4, NOTE_Ds4};  // G# - C - D# - F#
const float A7major[6] = {      0, NOTE_A2, NOTE_E3, NOTE_G3, NOTE_Cs4,NOTE_E4};  // A - C# - E - G
const float As7major[6] = {      0, NOTE_As2, NOTE_F3, NOTE_Gs3, NOTE_D4,NOTE_F4};  // A# - D - F - G#
const float B7major[6] = {      0, NOTE_B2, NOTE_Fs3,NOTE_A3, NOTE_Ds4,NOTE_Fs4}; // B - D# - F# - A

const float Caug[6] = {      0, NOTE_C3, NOTE_E3, NOTE_Gs3, NOTE_C4, NOTE_E4};  // C - E - G#
const float Csaug[6] = {      0, NOTE_Cs3, NOTE_F3, NOTE_A3, NOTE_Cs4, NOTE_F4};  // C# - F - A
const float Daug[6] = {      0,       0, NOTE_D3, NOTE_As3, NOTE_D4, NOTE_Fs4}; // D - F# - A#
const float Dsaug[6] = {      0,       0, NOTE_Ds3, NOTE_B3, NOTE_Ds4, NOTE_G4}; // D# - G - B
const float Eaug[6] = {NOTE_E2, NOTE_C3, NOTE_E3, NOTE_Gs3,NOTE_C4, NOTE_E4};  // E - G# - C
const float Faug[6] = {      0, NOTE_A2, NOTE_F3, NOTE_A3, NOTE_Cs4, NOTE_F4};  // F - A - C#
const float Fsaug[6] = {      0, NOTE_As2, NOTE_Fs3, NOTE_As3, NOTE_D4, NOTE_Fs4};  // F# - A# - D
const float Gaug[6] = {NOTE_G2, NOTE_B2, NOTE_Ds3, NOTE_G3, NOTE_B3, NOTE_Ds4};  // G - B - D#
const float Gsaug[6] = {NOTE_Gs2, NOTE_C3, NOTE_E3, NOTE_Gs3, NOTE_C4, NOTE_E4};  // G# - C - E
const float Aaug[6] = {      0, NOTE_A2, NOTE_F3, NOTE_A3, NOTE_Cs4,NOTE_F4};  // A - C# - F
const float Asaug[6] = {      0, NOTE_As2, NOTE_Fs3, NOTE_As3, NOTE_D4,NOTE_Fs4};  // A# - D - F#
const float Baug[6] = {      0, NOTE_B2, NOTE_G3,NOTE_B3, NOTE_Ds4,NOTE_G4}; // B - D# - G

//                   E2, F2, F2#, G2, G2#, A2, A2#, B2
// C3, C3#, D3, D3#, E3, F3, F3#, G3, G3#, A3, A3#, B3
// C4, C4#, D4, D4#, E4, F4, F4#, G4, G4#, A4, A4#, B4
