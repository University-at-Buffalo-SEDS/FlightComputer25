#include "util/avghistory.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

void AvgHistory_Init(AvgHistory *ah) {
    // Initialize history to zeros.
    for (size_t i = 0; i < AVG_HISTORY_LENGTH; i++) {
        ah->avg_history[i] = 0.0f;
    }
    ah->hist_len = 0;
    ah->count = 0;
}

void AvgHistory_Add(AvgHistory *ah, float reading) {
    // If the number of samples in the current average is more than the threshold,
    // shift the history, commit the average, and reset the count.
    if (ah->count > AVG_HISTORY_SAMPLES) {
        for (int i = AVG_HISTORY_LENGTH - 1; i > 0; i--) {
            ah->avg_history[i] = ah->avg_history[i - 1];
        }
        ah->count = 1;  // Reset count; we start a new average with the current reading.
        ah->hist_len = MIN(AVG_HISTORY_LENGTH - 1, ah->hist_len + 1);
    }
    // Update the running average stored in avg_history[0]:
    // new_average = (current_average * count + new_reading) / (count + 1)
    ah->avg_history[0] = (ah->avg_history[0] * ah->count + reading) / (ah->count + 1);
    ah->count++;
}

int AvgHistory_Full(AvgHistory *ah) {
    // Consider the history full if we have at least (AVG_HISTORY_LENGTH-1) committed averages.
    return (ah->hist_len >= (AVG_HISTORY_LENGTH - 1));
}

float AvgHistory_OldAvg(AvgHistory *ah) {
    // Returns the oldest committed average in the history.
    return ah->avg_history[AVG_HISTORY_LENGTH - 1];
}
