#include "gatecar_api.h"
int gatecar_api_version(void) { return GATECAR_API_VERSION; }
int gatecar_input_size(void) { return sizeof(GateCarInput); }
int gatecar_output_size(void) { return sizeof(GateCarOutput); }
