// util/avghistory.h
#ifndef AVGHISTORY_H
#define AVGHISTORY_H

#include <stdint.h>
#include <stddef.h>

// Number of readings per chunk
#ifndef AVG_HISTORY_SAMPLES
#define AVG_HISTORY_SAMPLES 10
#endif

// We only need one “old” chunk + the current chunk
#ifndef AVG_HISTORY_LENGTH
#define AVG_HISTORY_LENGTH 2
#endif

typedef struct {
    float    avg_history[AVG_HISTORY_LENGTH];
    uint8_t  hist_len;  // how many old chunks have been committed (max = AVG_HISTORY_LENGTH-1)
    uint8_t  count;     // how many samples have gone into avg_history[0] so far
} AvgHistory;

void    AvgHistory_Init(    AvgHistory *ah);
void    AvgHistory_Add(     AvgHistory *ah, float reading);
int     AvgHistory_Full(    const AvgHistory *ah);
float   AvgHistory_OldAvg(  const AvgHistory *ah);

#endif // AVGHISTORY_H
