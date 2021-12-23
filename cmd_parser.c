#include <stdio.h>
#include "cmd_parser.h"
#include "led_param.h"
extern uint32_t test_pattern;
extern int32_t led_select;
extern int32_t led_lighting_mode;
extern uint8_t led_patterns[LED_PORTS][LED_NUM][LED_CHANNELS];
extern uint32_t led_total_width;
extern uint32_t led_total_height;

uint8_t br_level = 0x40;
uint32_t led_color = RED;
int cmd_parser(uint8_t *buf){
	char cmd[64];
    int i = 0;
    int j = 0;
    int k = 0;
    printf("%s\n", __func__);

    if(strstr(buf, "set_br")){
    	sscanf(buf, "%s %d", cmd, &br_level);
	    printf("br_level= 0%x\n", br_level);
    }

    if(strstr(buf, "led_select")){
       sscanf(buf, "%s %d", cmd, &led_select);
       printf("led_select:%d\n", led_select);
    }
    
    if(strstr(buf, "set_total_width")){
       sscanf(buf, "%s %d", cmd, &led_total_width);
       printf("led_total_width:%d\n", led_total_width);
    }
    
    if(strstr(buf, "set_total_height")){
       sscanf(buf, "%s %d", cmd, &led_total_height);
       printf("led_total_height:%d\n", led_total_height);
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
    }else if(!strcmp("test_mode:normal", buf)){
        led_lighting_mode = LED_NORMAL_MODE;
    }else if(!strcmp("test_mode:area", buf)){
        led_lighting_mode = LED_AREA_MODE;    
    }	   
    
#ifdef LED_1111
    //depreciated, need to implement new method
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
        for(j = 0; j < LED_PORTS; j++){    
            for(i = 0; i < LED_NUM; i++){
                for(k = 0; k < LED_CHANNELS; k++){
                    led_patterns[j][i][k] = br_level; 
                }
            }
        }
    }else if(led_color == RED){
        for(j = 0; j < LED_PORTS; j++){    
            for(i = 0; i < LED_NUM; i++){
                for(k = 0; k < LED_CHANNELS; k++){
                    if(k == 1){
                        led_patterns[j][i][k] = br_level; 
                    }else{
                        led_patterns[j][i][k] = 0x00;  
                    }
                }
            }
        }

    }else if(led_color == GREEN){
        for(j = 0; j < LED_PORTS; j++){    
            for(i = 0; i < LED_NUM; i++){
                for(k = 0; k < LED_CHANNELS; k++){
                    if(k == 0){
                        led_patterns[j][i][k] = br_level; 
                    }else{
                        led_patterns[j][i][k] = 0x00;  
                    }
                }
            }
        }
    
    }else if(led_color == BLUE){
        for(j = 0; j < LED_PORTS; j++){    
            for(i = 0; i < LED_NUM; i++){
                for(k = 0; k < LED_CHANNELS; k++){
                    if(k == 2){
                        led_patterns[j][i][k] = br_level; 
                    }else{
                        led_patterns[j][i][k] = 0x00;  
                    }
                }
            }
        }
    
    }

#endif
    printf("test_pattern = 0x%x\n", test_pattern);
    return 0;
}
