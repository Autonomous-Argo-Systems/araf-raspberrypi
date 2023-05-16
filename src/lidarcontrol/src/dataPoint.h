#ifndef DATA_POINT_CPP
#define DATA_POINT_CPP

typedef struct __attribute__((packed))
{
    float x,y,z;
    float intensity;
    uint16_t ring;
    float time;
} DataPoint;

#endif