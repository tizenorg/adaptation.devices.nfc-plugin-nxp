
#include <dlog.h>

#define LOG_COLOR_RED 		"\033[0;31m"
#define LOG_COLOR_GREEN 	"\033[0;32m"
#define LOG_COLOR_BROWN 	"\033[0;33m"
#define LOG_COLOR_BLUE 		"\033[0;34m"
#define LOG_COLOR_PURPLE 	"\033[0;35m"
#define LOG_COLOR_CYAN 		"\033[0;36m"
#define LOG_COLOR_LIGHTBLUE "\033[0;37m"
#define LOG_COLOR_END		"\033[0;m"

#define LOGE(format,args...) \
do {\
	LOGD(LOG_COLOR_BROWN"[file = %s : method = %s : line = %d]"LOG_COLOR_END, basename(__FILE__), __func__, __LINE__); \
	LOGD(LOG_COLOR_BROWN""format""LOG_COLOR_END, ##args);\
}while(0);

