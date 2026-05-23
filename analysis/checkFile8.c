#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define FILE_BASE_NAME "brauciens"
#define MAX_FILENAME_LEN 256

#pragma pack(push, 1)
struct ADCRecord_Octa
{
    uint32_t timestamp_us;
    int32_t raw_value1; // ADC1 AIN0/1
    int32_t raw_value2; // ADC1 AIN2/3
    int32_t raw_value3; // ADC2 AIN0/1
    int32_t raw_value4; // ADC2 AIN2/3
    int32_t raw_value5; // ADC3 AIN0/1
    int32_t raw_value6; // ADC3 AIN2/3
    int32_t raw_value7; // ADC4 AIN0/1
    int32_t raw_value8; // ADC4 AIN2/3
};
#pragma pack(pop)

int main()
{
    FILE *fptr;
    int record_count = 0;
    int file_num = -1;
    char file_name[MAX_FILENAME_LEN];
    char file_number[12];

    strcpy(file_name, FILE_BASE_NAME);

    printf("%s", "Please enter the number of brauciens_x.bin (-1 for brauciens.bin)\n");
    scanf("%d", &file_num);

    if (file_num == -1)
    {
        strcat(file_name, ".bin");
    }
    else
    {
        sprintf(file_number, "_%d", file_num);
        strcat(file_name, file_number);
        strcat(file_name, ".bin");
    }

    printf("Reading from file: %s\n", file_name);

    fptr = fopen(file_name, "rb"); // open in read-binary mode

    if (fptr == NULL)
    {
        perror("The file is not opened");
        return 1;
    }

    struct ADCRecord_Octa record;
    FILE *files[8];

    for (int i = 0; i < 8; i++) {
        char out_filename[20];
        sprintf(out_filename, "output%d.txt", i + 1);
        files[i] = fopen(out_filename, "w");
        if (files[i] == NULL) {
            printf("Cannot create output file %s.\n", out_filename);
            for (int j = 0; j < i; j++) {
                fclose(files[j]);
            }
            fclose(fptr);
            return 1;
        }
    }

    while (fread(&record, sizeof(struct ADCRecord_Octa), 1, fptr) == 1)
    {
        record_count++;
        fprintf(files[0], "Raw=%d, Time=%u\n", record.raw_value1, record.timestamp_us);
        fprintf(files[1], "Raw=%d, Time=%u\n", record.raw_value2, record.timestamp_us);
        fprintf(files[2], "Raw=%d, Time=%u\n", record.raw_value3, record.timestamp_us);
        fprintf(files[3], "Raw=%d, Time=%u\n", record.raw_value4, record.timestamp_us);
        fprintf(files[4], "Raw=%d, Time=%u\n", record.raw_value5, record.timestamp_us);
        fprintf(files[5], "Raw=%d, Time=%u\n", record.raw_value6, record.timestamp_us);
        fprintf(files[6], "Raw=%d, Time=%u\n", record.raw_value7, record.timestamp_us);
        fprintf(files[7], "Raw=%d, Time=%u\n", record.raw_value8, record.timestamp_us);
    }

    printf("Processed %d records into output1.txt through output8.txt\n", record_count);

    for (int i = 0; i < 8; i++) {
        fclose(files[i]);
    }
    fclose(fptr);

    return 0;
}
