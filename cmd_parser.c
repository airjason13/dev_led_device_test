#include <stdio.h>
#include "cmd_parser.h"
#include "led_param.h"
extern uint32_t test_pattern;
extern int32_t led_select;
uint8_t br_level = 0x40;
uint32_t led_color = RED;
int cmd_parser(uint8_t *buf){
	char cmd[64];
    printf("%s\n", __func__);

    if(strstr(buf, "set_br")){
    	sscanf(buf, "%s %d", cmd, &br_level);
	    printf("br_level= 0%x\n", br_level);
    }

    if(strstr(buf, "led_select")){
       sscanf(buf, "%s %d", cmd, &led_select);
       printf("led_select:%d\n", led_select);
    }
    

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
#ifdef LED_1111
    if(led_color == WHITE){
    	test_pattern = br_level << 16 | br_level << 8 | br_level;
    }else if(led_color == RED){
    	test_pattern = br_level << 16 & 0xff0000;
    }else if(led_color == GREEN){
    	test_pattern = br_level << 8 & 0xff00;
    }else if(led_color == BLUE){
    	test_pattern = br_level  & 0xff;
    }
#else
    if(led_color == WHITE){
    	test_pattern = br_level << 16 | br_level << 8 | br_level;
    }else if(led_color == RED){
    	test_pattern = br_level << 8 & 0xff00;	
    }else if(led_color == GREEN){
    	test_pattern = br_level << 16 & 0xff0000;
    }else if(led_color == BLUE){
    	test_pattern = br_level  & 0xff;
    }

#endif
    printf("test_pattern = 0x%x\n", test_pattern);
    return 0;
}
