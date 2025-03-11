#include <stdio.h>
#include <stdint.h>
#include <string.h>


uint16_t calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; 
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;  
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {  
                crc = (crc << 1) ^ 0x1021; 
            } else {
                crc = crc << 1;  
            }
            crc &= 0xFFFF; 
        }
    }
    
    return crc;
}

void verify_data(const char* data) {
    size_t len = strlen(data);
    char* data_copy = (char*)malloc(len + 1);
    strcpy(data_copy, data);
    
    // remove \r\n
    data_copy[len-2] = '\0';
    len -= 2;
    
    // get crc16 
    char crc_str[5] = {0};
    strncpy(crc_str, data_copy + len - 4, 4);
    data_copy[len-4] = '\0'; 
    
    // remove trailing space if exists
    len = strlen(data_copy);
    if (len > 0 && data_copy[len-1] == ' ') {
        data_copy[len-1] = '\0';
    }
    
    // transfer received crc16
    uint16_t received_crc;
    sscanf(crc_str, "%4hx", &received_crc);
    
    // calculate crc16
    uint16_t calc_crc = calculate_crc16((uint8_t*)data_copy, strlen(data_copy));
    printf("data: %s\n", data_copy);
    printf("received crc16: 0x%04X\n", received_crc);
    printf("calculate crc16: 0x%04X\n", calc_crc);
    printf("verify result: %s\n", (received_crc == calc_crc) ? "pass" : "fail");
    
    free(data_copy);
}

char* generate_crc16(const char* data) {
    // Calculate CRC16
    uint16_t crc = calculate_crc16((const uint8_t*)data, strlen(data));
    
    // Allocate memory for result string (original + space + 4 chars for CRC + \r\n)
    char* result = (char*)malloc(strlen(data) + 8);
    if (!result) return NULL;
    
    // Copy original data
    strcpy(result, data);
    
    // Add space if not exists at the end
    size_t len = strlen(result);
    if (len > 0 && result[len-1] != ' ') {
        strcat(result, " ");
    }
    
    // Add CRC16 value
    char crc_str[5];
    snprintf(crc_str, sizeof(crc_str), "%04X", crc);
    strcat(result, crc_str);
    
    // Add \r\n
    strcat(result, "\r\n");
    
    return result;
}

// test
void test_crc16(void) {
    // 测试用例1：字符串格式命令
    const char* cmd1 = "cmd 01 08 78623 369707 83986 391414 508006 455123";
    uint16_t crc1 = calculate_crc16((const uint8_t*)cmd1, strlen(cmd1));
    printf("Test 1: '%s'\n", cmd1);
    printf("CRC16: 0x%04X\n\n", crc1);
    
    // 测试用例2：带CRC的完整命令
    const char* cmd2 = "cmd 01 08 78623 369707 83986 391414 508006 455123 B9FC\r\n";
    printf("Test 2:\n");
    verify_data(cmd2);
    
    // 测试用例3：生成带CRC的命令
    printf("Test 3:\n");
    char* cmd3 = generate_crc16(cmd1);
    if (cmd3) {
        printf("Generated command: %s", cmd3);
        verify_data(cmd3);
        free(cmd3);
    }
}

int main(void) {
    test_crc16();
    return 0;
} 
