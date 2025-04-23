#ifndef AVGHISTORY_H
#define AVGHISTORY_H

#include <stdint.h>
#include <stddef.h>

// Define the parameters; adjust these as needed.
#ifndef AVG_HISTORY_SAMPLES
#define AVG_HISTORY_SAMPLES 10
#endif

#ifndef AVG_HISTORY_LENGTH
#define AVG_HISTORY_LENGTH 3
#endif

// The AvgHistory structure holds a history of running averages.
typedef struct {
    float avg_history[AVG_HISTORY_LENGTH]; // History array.
    uint8_t hist_len;   // Number of committed averages (maximum AVG_HISTORY_LENGTH-1).
    uint8_t count;      // Number of samples collected for the current average.
} AvgHistory;

// Initializes an AvgHistory instance.
void AvgHistory_Init(AvgHistory *ah);

// Adds a new reading, updating the running average.
// When more than AVG_HISTORY_SAMPLES have been added, it "commits" the current average
// by shifting the history and resetting the sample counter.
void AvgHistory_Add(AvgHistory *ah, float reading);

// Returns nonzero if enough samples have been added such that the oldest average is valid.
int AvgHistory_Full(AvgHistory *ah);

// Returns the oldest average stored in the history.
float AvgHistory_OldAvg(AvgHistory *ah);

#endif
