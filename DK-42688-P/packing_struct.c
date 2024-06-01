#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdalign.h>
typedef struct {
    float gyro[3];
    float acc[3];
    float quat[4];
    float temperature;
} HR_IMUdata;

typedef union {
    float data[11];
    char bytes[sizeof(float) * 11];
} HR_IMUdata_union;

static HR_IMUdata hr_data_array[10] = {0};
static const int hr_data_array_size = sizeof(hr_data_array) / sizeof(HR_IMUdata);
static const int hr_data_array_byte_size = sizeof(hr_data_array);

unsigned char hr_sendbuf[sizeof(hr_data_array)];
static unsigned int hr_data_counter = 0;

void setData(const HR_IMUdata* data)
{
    hr_data_array[hr_data_counter % hr_data_array_size] = *data;
    hr_data_counter++;
}

unsigned int writeDataToBuff(unsigned char* buff)
{
    if(hr_data_counter > hr_data_array_size)
    {
        const int first_index = hr_data_counter % hr_data_array_size;
        const int first_size = hr_data_array_size - first_index;
        const int write_size = first_size * sizeof(HR_IMUdata);
        memcpy(buff, &hr_data_array[first_index], write_size);
        printf("if first %d %d\n", first_index,write_size);
        memcpy(buff + write_size, hr_data_array, (hr_data_array_size - first_size) * sizeof(HR_IMUdata));
        printf("if %d\n", (hr_data_array_size - first_size) * sizeof(HR_IMUdata));
    }
    else if(hr_data_counter == hr_data_array_size)
    {
        memcpy(buff, &hr_data_array, sizeof(hr_data_array));
        printf("else if %d\n", sizeof(hr_data_array));
    }
    else
    {
        memcpy(buff, &hr_data_array, sizeof(HR_IMUdata) * hr_data_counter);
        printf("else %d\n", sizeof(HR_IMUdata) * hr_data_counter);
    }
    unsigned int ret = (hr_data_counter > hr_data_array_size) ? hr_data_array_size : hr_data_counter;
    hr_data_counter = 0;
    return ret;
}

typedef struct 
{
    int value; // aligns on a 4-byte boundary. There will be 28 bytes of padding between value and alignas
    char alignedMemory[32]; // assuming a 32 byte friendly cache alignment
} cacheFriendly; 

int main(int argc, char const *argv[])
{
    printf("sizeof(HR_IMUdata) = %d\n", sizeof(HR_IMUdata));
    printf("sizeof(float) * 11 = %d\n", sizeof(float) * 11);
    printf("sizeof(hr_data_array) = %d\n", sizeof(hr_data_array));
    memset(hr_data_array, 0, sizeof(hr_data_array));
    memset(hr_sendbuf, 0, sizeof(hr_sendbuf));
    HR_IMUdata data = {};
    data.gyro[0] = 3;
    data.temperature = 3;
    setData(&data);
    for(int i = 0; i < 45; i++)
    {
        data.gyro[0] = i;
        data.temperature = i;
        setData(&data);
    }
    data.gyro[0] = 0;
    data.temperature = 0;
    setData(&data);
    setData(&data);
    data.gyro[0] = 10;
    data.temperature = 10;
    setData(&data);
    unsigned int size = writeDataToBuff(hr_sendbuf);
    printf("Data size is %u = %u\n", size, size * sizeof(HR_IMUdata));
    printf("[row %5d ] ", 0);
    for (int i = 0; i < sizeof(hr_data_array); i++) {
        printf("%02X ", hr_sendbuf[i]);
        if((i + 1)% sizeof(HR_IMUdata) == 0)
        {
            printf("\n");
            printf("[row %5d ] ", i);
        }
    }
    printf("\n");
    memset(hr_sendbuf, 0, sizeof(hr_sendbuf));

    data.gyro[0] = 10;
    data.temperature = 10;
    HR_IMUdata_union data_union = {};
    assert(sizeof(HR_IMUdata) == sizeof(HR_IMUdata_union));
    printf("sizeof(HR_IMUdata) = %d\n", sizeof(HR_IMUdata));
    printf("sizeof(HR_IMUdata_union) = %d\n", sizeof(HR_IMUdata_union));
    memcpy(&data_union, &data, sizeof(HR_IMUdata));

    printf("[row %5d ] ", 0);
    for (int i = 0; i < sizeof(HR_IMUdata_union); i++) {
        printf("%02X ", data_union.bytes[i]);
    }
    printf("\n");
    printf("alignof(HR_IMUdata) = %d\n", alignof(HR_IMUdata));
    printf("alignof(HR_IMUdata_union) = %d\n", alignof(HR_IMUdata_union));
    printf("alignof(cacheFriendly) = %d\n", alignof(cacheFriendly));
    return 0;
}