#include "data_path.h"

typedef struct mmPoints_t
{
    float x;
    float y;
    float z;
    float intensity;
} mmPoints;

extern void combineNeighborsDataPathObj(MmwDemo_DataPathObj *dataPathObj, unsigned int *nbOutput, mmPoints *output);
