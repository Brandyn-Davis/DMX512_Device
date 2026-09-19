#ifndef FIELDS_H
#define FIELDS_H

bool isAlpha(char c);
bool isNumeric(char c);
int32_t _atoi(char* field);
bool isCommand(char* buffer, uint8_t fieldCount, const char* strCommand, uint8_t minArguments);
char* getFieldString(char* buffer, uint8_t* fieldPos, uint8_t fieldCount, uint8_t fieldNumber);
int32_t getFieldInteger(char* buffer, uint8_t* fieldPos, uint8_t fieldCount, uint8_t fieldNumber);
void parseFields(char* buffer, char* fieldType, uint8_t* fieldPos, uint8_t* fieldCount, uint8_t maxFields, uint8_t maxChars);

#endif
