#ifndef CONFGENERATOR_H_
#define CONFGENERATOR_H_

#include "datatypes.h"
#include <stdint.h>
#include <stdbool.h>
#include "buffer.h"

// Functions


int32_t confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf);

bool confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf);



// CONFGENERATOR_H_
#endif