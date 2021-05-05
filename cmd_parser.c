#include <stdio.h>
#include "cmd_parser.h"

extern uint32_t test_pattern;
uint8_t br_level = 0x40;
uint32_t led_color = RED;
int cmd_parser(uint8_t *buf){
    printf("%s\n", __func__);
    printf("test_pattern = 0x%x\n", test_pattern);

#if 1
    if(strstr(buf, "set_br")){
        //printf("buf = %s\n", buf);	
	char cmd[64];
	unsigned int test_br = 0;
    	sscanf(buf, "%s %d", cmd, &br_level);
	//printf("cmd:%s\n", cmd);
	//printf("test_br= 0x%x\n", test_br);
	printf("br_level= 0%x\n", br_level);
    }
#else
    if(strstr(buf, "set_br:")){
        //printf("buf = %s\n", buf);	
	char cmd[64];
	unsigned int test_br = 0;
    	sscanf(strstr(buf, "set_br:"), " %d", cmd, &br_level);
	//printf("cmd:%s\n", cmd);
	//printf("test_br= 0x%x\n", test_br);
	printf("br_level= 0%x\n", br_level);
    }
#endif    

    if(!strcmp("test_color:white", buf)){
	led_color = WHITE;    
    	//test_pattern = br_level << 16 | br_level << 8 | br_level;
    }else if(!strcmp("test_color:red", buf)){
	led_color = RED;    
    	//test_pattern = br_level << 16 & 0xff0000;
    }else if(!strcmp("test_color:green", buf)){
	led_color = GREEN;    
    	//test_pattern = br_level << 8 & 0xff00;
    }else if(!strcmp("test_color:blue", buf)){
	led_color = BLUE;    
    	//test_pattern = br_level  & 0xff;
    }	    
    if(led_color == WHITE){
    	test_pattern = br_level << 16 | br_level << 8 | br_level;
    }else if(led_color == RED){
    	test_pattern = br_level << 16 & 0xff0000;
    }else if(led_color == GREEN){
    	test_pattern = br_level << 8 & 0xff00;
    }else if(led_color == BLUE){
    	test_pattern = br_level  & 0xff;
    }
    return 0;
}
