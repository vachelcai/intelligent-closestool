#include "eeprom.h"

void readData(uint32_t dataAdd,uint32_t add,uint32_t lenght){
	for(int i=0;i<lenght;i++){
		*(dataAdd+i)=*(add+1);
	}
}

void saveData(uint32