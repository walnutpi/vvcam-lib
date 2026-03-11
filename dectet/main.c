#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "get_frame.h"

#define DEV_ISP "/proc/vsi/isp_subdev0"

struct sensor_setting
{
    const char *sensor;
    const char *mode;
    const char *xml;
    const char *manu_json;
    const char *auto_json;
};

struct sensor_setting gc2093 = {
    .sensor = "gc2093",
    .mode = "0",
    .xml = "/etc/vvcam/gc2093-1920x1080.xml",
    .manu_json = "/etc/vvcam/gc2093-1920x1080_manual.json",
    .auto_json = "/etc/vvcam/gc2093-1920x1080_auto.json"};

// 打开文件路径 DEV_ISP , 写入指定的配置参数，完成isp的配置
void write_isp_setting(struct sensor_setting *setting)
{
    FILE *fp;
    
    fp = fopen(DEV_ISP, "w");
    if (fp == NULL) {
        printf("Failed to open %s\n", DEV_ISP);
        return;
    }
    fprintf(fp, "0 sensor=%s\n", setting->sensor);
    fclose(fp);
    
    fp = fopen(DEV_ISP, "w");
    fprintf(fp, "0 mode=%s\n", setting->mode);
    fclose(fp);
    
    fp = fopen(DEV_ISP, "w");
    fprintf(fp, "0 xml=%s\n", setting->xml);
    fclose(fp);
    
    fp = fopen(DEV_ISP, "w");
    fprintf(fp, "0 manu_json=%s\n", setting->manu_json);
    fclose(fp);
    
    fp = fopen(DEV_ISP, "w");
    fprintf(fp, "0 auto_json=%s\n", setting->auto_json);
    fclose(fp);
}

int main(int argc, char *argv[])
{

    write_isp_setting(&gc2093);

    int ret = get_frame();
    if (ret < 0)
    {
        printf("get frame failed\n");
        return -1;
    }
    return 0;
}