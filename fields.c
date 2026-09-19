#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

bool isAlpha(char c) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))
        return 1;
    else
        return 0;
}

bool isNumeric(char c) {
    if (c >= '0' && c <= '9')
        return 1;
    else
        return 0;
}

int32_t _atoi(char* field) {
    int32_t result = 0;

    int i;
    for (i = 0; field[i] != '\0'; i++) {
        result = result*10 + (field[i] - 48);
    }

    return result;
}


bool isCommand(char* buffer, uint8_t fieldCount, const char* strCommand, uint8_t minArguments) {
    if (minArguments >= fieldCount) return 0;

    int i;
    for (i = 0; buffer[i] != '\0'; i++) {
        if (buffer[i] != strCommand[i]) return 0;
    }

    return 1;
}

char* getFieldString(char* buffer, uint8_t* fieldPos, uint8_t fieldCount, uint8_t fieldNumber) {
    if (fieldNumber >= fieldCount) return NULL;
    return buffer + fieldPos[fieldNumber];
}

int32_t getFieldInteger(char* buffer, uint8_t* fieldPos, uint8_t fieldCount, uint8_t fieldNumber) {
    if (fieldNumber >= fieldCount) return 0;

    int i = 0;
    char c = buffer[i+fieldPos[fieldNumber]];
    while (c != '\0') {
        if (!isNumeric(c)) return 0;
        i++;
        c = buffer[i+fieldPos[fieldNumber]];
    }

    return _atoi(buffer + fieldPos[fieldNumber]);
}

void parseFields(char* buffer, char* fieldType, uint8_t* fieldPos, uint8_t* fieldCount, uint8_t maxFields, uint8_t maxChars) {
    char prevChr = '\0';
    char currChr;

    int i = 0;
    while (i < maxChars) {
        currChr = buffer[i];

        if (prevChr == '\0' && (isAlpha(currChr) || isNumeric(currChr))) {
            if (isAlpha(currChr))
                fieldType[*fieldCount] = 'a';
            else if (isNumeric(currChr))
                fieldType[*fieldCount] = 'n';
            fieldPos[*fieldCount] = i;
            *fieldCount = *fieldCount + 1;
        }
        else if (!(isAlpha(currChr) || isNumeric(currChr))) {
            if (currChr == '\0') return;
            buffer[i] = '\0';
            currChr = '\0';
        }
        else if (currChr == '\0') {
            return;
        }

        if (*fieldCount >= maxFields) return;

        prevChr = currChr;
        i++;
    }
}
