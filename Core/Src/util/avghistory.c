// util/avghistory.c
#include "util/avghistory.h"
#define MIN(a,b) (((a)<(b))?(a):(b))

void AvgHistory_Init(AvgHistory *ah) {
    for (size_t i = 0; i < AVG_HISTORY_LENGTH; i++) {
        ah->avg_history[i] = 0.0f;
    }
    ah->hist_len = 0;
    ah->count    = 0;
}

void AvgHistory_Add(AvgHistory *ah, float reading) {
    // If we've just collected the 10th sample, commit that chunk:
    if (ah->count >= AVG_HISTORY_SAMPLES) {
        // shift the old averages up one slot
        for (int i = AVG_HISTORY_LENGTH - 1; i > 0; --i) {
            ah->avg_history[i] = ah->avg_history[i - 1];
        }
        // we’ve now committed one more chunk:
        ah->hist_len = MIN((uint8_t)(AVG_HISTORY_LENGTH - 1),
                           (uint8_t)(ah->hist_len + 1));
        // start a fresh chunk (reset count and zero the accumulator)
        ah->count          = 0;
        ah->avg_history[0] = 0.0f;
    }

    // incorporate this sample into the running average for the current chunk
    ah->avg_history[0] = (ah->avg_history[0] * ah->count + reading)
                       / (float)(ah->count + 1);
    ah->count++;
}

int   AvgHistory_Full(   const AvgHistory *ah) {
    // full once we've committed at least 1 chunk
    return ah->hist_len >= (AVG_HISTORY_LENGTH - 1);
}

float AvgHistory_OldAvg( const AvgHistory *ah) {
    // the sole “old” slot lives at index 1
    return ah->avg_history[AVG_HISTORY_LENGTH - 1];
}
