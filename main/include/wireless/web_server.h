#pragma once

// macros to concatenate strings with defines
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void start_webserver();
