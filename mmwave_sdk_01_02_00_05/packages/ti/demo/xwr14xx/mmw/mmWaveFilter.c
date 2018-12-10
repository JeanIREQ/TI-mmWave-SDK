#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "mmWaveFilter.h"

#define MAXPTS 150

mmPoints pointsTable[25][25];
unsigned char nbRow;
unsigned char nbCol[25];
mmPoints pointsInput[MAXPTS];
unsigned int nbPointsInput;

static void copyPoint(mmPoints *input, mmPoints *output)
{
    output->x = input->x;
    output->y = input->y;
    output->z = input->z;
    output->intensity = input->intensity;
}

static int copyVector(unsigned int nbInput, mmPoints *input, mmPoints *output)
{
    unsigned int i;

    for(i = 0; i < nbInput; i++)
        copyPoint(&input[i], &output[i]);

    return nbInput;
}

static int compareIntensity(const void *a, const void *b)
{
    mmPoints *p1 = (mmPoints *)a;
    mmPoints *p2 = (mmPoints *)b;

    if(p1->intensity < p2->intensity)
        return 1;
    if(p1->intensity > p2->intensity)
        return -1;
    return 0;
}

static double distance(mmPoints *p1, mmPoints *p2)
{
    return sqrt((p1->x - p2->x)*(p1->x - p2->x) + 
                (p1->y - p2->y)*(p1->y - p2->y) + 
                (p1->z - p2->z)*(p1->z - p2->z));
}

static mmPoints meanFromIntensity(int row)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double intensity = 0.0;
    unsigned int i;

    for(i = 0; i < nbCol[row]; i++)
    {
        x += pointsTable[row][i].x * pointsTable[row][i].intensity;
        y += pointsTable[row][i].y * pointsTable[row][i].intensity;
        z += pointsTable[row][i].z * pointsTable[row][i].intensity;
        intensity += pointsTable[row][i].intensity;
    }
    mmPoints out;
    out.x = x/intensity;
    out.y = y/intensity;
    out.z = z/intensity;
    out.intensity = pointsTable[row][0].intensity;

    return out;
}

static double distanceFromRadar(mmPoints *in)
{
    double d = in->x;
    if(d < in->y)
        d = in->y;
    if(d < in->z)
        d = in->z;

    return d;
}

static int combineNeighbors(unsigned int nbInput, mmPoints *input, double maxDistance, unsigned int *nbOutput, mmPoints *output)
{
    mmPoints copy[MAXPTS];
    int nbCopy;
    unsigned int i;
    unsigned int row;
    unsigned int total;
    memset((void*)&pointsTable, 0, sizeof(pointsTable));
    memset((void*)&nbCol, 0, sizeof(nbCol));
    nbRow = 0;

    if(nbInput <= 1)
    {
        *nbOutput = copyVector(nbInput, input, output);
        return nbInput;
    }

    if(maxDistance < 0.001)
    {
        *nbOutput = copyVector(nbInput, input, output);
        return nbInput;
    }

    nbCopy = copyVector(nbInput, input, copy);

    qsort(copy, nbCopy, sizeof(mmPoints), compareIntensity);

    /* points are sorted from highest to lowest intensity. */

    for(i = 0; i < nbCopy; i++)
    {
        int clusterIdx = -1;
        double dMin = 100.0;
        for(row = 0; row < nbRow; row++)
        {
            unsigned int col;
            for(col = 0; col < nbCol[row]; col++)
            {
                double d = distance(&copy[i], &pointsTable[row][col]);
                if(dMin > d)
                {
                    dMin = d;
                    clusterIdx = row;
                }
            }
        }

        if((clusterIdx == -1) || (dMin > maxDistance))
        {
            if((nbRow < 25) && (nbCol[nbRow] < 25))
            {
                copyPoint(&copy[i], &pointsTable[nbRow][nbCol[nbRow]]);
                nbCol[nbRow]++;
                nbRow++;
            }
        }
        else
        {
            if((clusterIdx < 25) && (nbCol[clusterIdx] < 25))
            {
                copyPoint(&copy[i], &pointsTable[clusterIdx][nbCol[clusterIdx]]);
                nbCol[clusterIdx]++;
            }
        }
    }

    total = 0;

    for(row = 0; row < nbRow; row++)
    {
        mmPoints s = meanFromIntensity(row);
        if(distanceFromRadar(&s) > 0.15)
        {
            if(s.intensity >= 20.0)
            {
                copyPoint(&s, &output[total]);
                total ++;
            }
        }
    }

    *nbOutput = total;

    /* keep point with high intensity */
    float thresholdMin = 0.8*output[0].intensity + 0.2*output[total-1].intensity;
    for(i = 0; i < total; i++)
        if(output[total-1-i].intensity < thresholdMin)
            (*nbOutput)--; /* 0nly shrink the size, no need to erase data, they will not be transmit */

    return *nbOutput;
}

static void convertTommPoints(MmwDemo_DataPathObj *dataPathObj, unsigned int *nbOutput, mmPoints *output)
{
    unsigned int i;
    *nbOutput = dataPathObj->numObjOut;
    if(*nbOutput > MAXPTS)
        *nbOutput = MAXPTS;

    /* replace by 512, was: unsigned int xyzQFormat = obj->xyzOutputQFormat; */

    for(i = 0; i < dataPathObj->numObjOut; i++)
    {
        output[i].intensity = 10.0*log10(dataPathObj->objOut[i].peakVal + 1.0);
        output[i].x = (float)dataPathObj->objOut[i].x / 512.0;
        output[i].y = (float)dataPathObj->objOut[i].y / 512.0;
        output[i].z = (float)dataPathObj->objOut[i].z / 512.0;
    }
}

void combineNeighborsDataPathObj(MmwDemo_DataPathObj *dataPathObj, unsigned int *nbOutput, mmPoints *output)
{
    convertTommPoints(dataPathObj, &nbPointsInput, pointsInput);
    combineNeighbors(nbPointsInput, pointsInput, 0.15, nbOutput, output);
}
