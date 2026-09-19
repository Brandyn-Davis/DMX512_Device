#ifndef EEPROM_H_
#define EEPROM_H_

void initEeprom(void);
void writeEeprom(uint16_t add, uint32_t data);
uint32_t readEeprom(uint16_t add);

#endif
