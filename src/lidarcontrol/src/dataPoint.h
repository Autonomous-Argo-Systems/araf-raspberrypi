#ifndef DATA_POINT_CPP
#define DATA_POINT_CPP

typedef struct __attribute__((packed))
{
    float x,y,z;
    float intensity;
    uint16_t ring;
    float time;
} DataPoint;

// DataPoint fetchDatapoint(std::vector<uint8_t> &data, uint32_t datapoint_offset){
//     // Setting up result buffer
//     DataPoint fetchedDatapoint;
//     uint8_t* fetchPtr = (uint8_t*) &fetchedDatapoint;

//     // Fetching pointdata
//     for (size_t i = 0; i < sizeof(DataPoint); i++)
//         fetchPtr[i] = data[i];
    
//     return fetchedDatapoint;
// }

#endif