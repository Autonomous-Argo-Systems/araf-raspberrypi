#include "dataPoint.h"

DataPoint fetchDatapoint(const std::vector<uint8_t> &data, uint32_t datapoint_offset){
    // Setting up result buffer
    DataPoint fetchedDatapoint;
    uint8_t* fetchPtr = (uint8_t*) &fetchedDatapoint;

    // Fetching pointdata
    for (size_t i = 0; i < sizeof(DataPoint); i++)
        fetchPtr[i] = data[datapoint_offset + i];
    
    return fetchedDatapoint;
}

void pushDatapoint(std::vector<uint8_t> &data, DataPoint &datapoint){
    // Setting up ptr
    uint8_t* fetchPtr = (uint8_t*) &datapoint;

    // Pushing pointdata
    for (size_t i = 0; i < sizeof(DataPoint); i++)
        data.push_back(fetchPtr[i]);
}