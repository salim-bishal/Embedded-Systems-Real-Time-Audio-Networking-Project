/*
 *  MSP432E401Y Enhanced Command Line Shell  v1.8.0
 *  Major upgrades over v1.7.1
 *     **New -gpio command**
 *         -gpio <idx> r            : read GPIO <idx>
 *         -gpio <idx> w <0|1>      : write value to GPIO <idx>
 *         -gpio <idx> t            : toggle GPIO <idx> (outputs only)
 *         idx User LEDs      (initial state from board config)
 *         idx 4     PK5  (starts LOW)
 *         idx 5     PD4  (starts HIGH)
 *         idx Side switches (read-only)
 *    **New -error command** prints internal error counters:
 *         unknown_cmd, overflow, bad_gpio, parse_gpio
 *     Help system extended ("-help gpio", "-help error").
 *     Code remains C89 ‘compliant (no mixed decls) for TI toolchain.
 *
 *  NOTE: Ensure SysConfig (ti_drivers_config.h) defines the following indices
 *        or edit `gpioMap[]` accordingly:
 *            CONFIG_GPIO_LED_0..3, CONFIG_GPIO_PK5, CONFIG_GPIO_PD4,
 *            CONFIG_GPIO_BUTTON_0, CONFIG_GPIO_BUTTON_1
 *        PK5/PD4 must be configured as outputs; buttons as inputs.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include "ti_drivers_config.h"

/* METADATA */
#define ABOUT_NAME        "Salim Sadman Bishal"
#define ABOUT_ASSIGNMENT  "ECE 5380 HWK1"
#define APP_VERSION       "v1.8.0"
#define BUILD_DATE        __DATE__
#define BUILD_TIME        __TIME__

/*  CONFIGURATION */
#define RX_BUF_SZ      64   /* total buffer size (includes headroom) */
#define MAX_CMD_LEN    32   /* 31 printable chars + 1 NUL          */

/* Index GPIO mapping (update if board config differs) */
static const uint_least8_t gpioMap[8] = {
    CONFIG_GPIO_LED_0,   /* 0 */
    CONFIG_GPIO_LED_1,   /* 1 */
    CONFIG_GPIO_LED_2,   /* 2 */
    CONFIG_GPIO_LED_3,   /* 3 */
    CONFIG_GPIO_PK5,     /* 4 */
    CONFIG_GPIO_PD4,     /* 5 */
    CONFIG_GPIO_BUTTON_0,/* 6 */
    CONFIG_GPIO_BUTTON_1 /* 7 */
};

/*   ERROR COUNTERS  */
enum { ERR_UNKNOWN_CMD, ERR_OVERFLOW, ERR_BAD_GPIO, ERR_PARSE_GPIO, NUM_ERR };
static unsigned errorCount[NUM_ERR] = {0};

/*   FILE SCOPE STATE  */
static UART_Handle gUart;
static char       gLineBuf[RX_BUF_SZ];
static size_t     len = 0;
static size_t     cursor = 0;

static char history[RX_BUF_SZ] = {0};
static bool hasHistory          = false;

/*  UTILITY I/O HELPERS  */
static void putStr(const char *s)       { UART_write(gUart, s, strlen(s)); }
static void putChar(char c)             { UART_write(gUart, &c, 1);         }
static void putHex32(uint32_t v)        { char b[11]; snprintf(b,11,"0x%08X",v); putStr(b);}
static void putDec(unsigned v)          { char b[12]; snprintf(b,12,"%u",v); putStr(b);}

/*   PROMPT & BANNER  */
static void prompt(void) { putStr("> "); }
static void banner(void)
{
    putStr("\r\n*** MSP432 Command Shell Ready ***\r\n");
    putStr("Type -help for a list of commands.\r\n\r\n");
    prompt();
}

/*  COMMAND IMPLEMENTATIONS  */
static void cmd_about(void)
{
    char msg[160];
    snprintf(msg,sizeof(msg),"%s | %s | %s | built %s %s\r\n",ABOUT_NAME,ABOUT_ASSIGNMENT,APP_VERSION,BUILD_DATE,BUILD_TIME);
    putStr(msg);
}

static void cmd_error(void)
{
    putStr("Errors:\r\n  unknown_cmd : "); putDec(errorCount[ERR_UNKNOWN_CMD]); putStr("\r\n");
    putStr("  overflow    : "); putDec(errorCount[ERR_OVERFLOW]);    putStr("\r\n");
    putStr("  bad_gpio    : "); putDec(errorCount[ERR_BAD_GPIO]);    putStr("\r\n");
    putStr("  parse_gpio  : "); putDec(errorCount[ERR_PARSE_GPIO]);  putStr("\r\n");
}

static void help_detail(const char *topic)
{
    if (!strcmp(topic,"help")) {
        putStr("-help [cmd]   : list all commands or details for <cmd>\r\n");
    } else if (!strcmp(topic,"about")) {
        putStr("-about        : show author, assignment, version, build date/time\r\n");
    } else if (!strcmp(topic,"print")) {
        putStr("-print text   : echo text exactly as entered\r\n");
    } else if (!strcmp(topic,"memr")) {
        putStr("-memr addrhex : read 32-bit word (flash 0x0-0x7FFFF | SRAM 0x20000000-0x2007FFFF)\r\n");
    } else if (!strcmp(topic,"gpio")) {
        putStr("-gpio idx op [val]\r\n"
               "  idx 0-3 : LEDs, 4:PK5, 5:PD4, 6-7: switches \r\n"
               "  op  r      : read pin\r\n"
               "      w v    : write 0/1\r\n"
               "      t      : toggle (outputs only)\r\n");
    } else if (!strcmp(topic,"error")) {
        putStr("-error       : show error counters since power-up\r\n");
    } else {
        putStr("No help for that topic\r\n");
    }
}

static void cmd_help(const char *args)
{
    if (!args || *args=='\0') {
        putStr("Commands: -help  -about  -print  -memr  -gpio  -error\r\nUse -help <cmd> for details.\r\n");
    } else {
        if (*args=='-') args++;
        help_detail(args);
    }
}

static void cmd_print(const char *text) { if(text) putStr(text); putStr("\r\n"); }

static bool addrOK(uint32_t a)
{ return (a<0x00080000u) || (a>=0x20000000u && a<0x20080000u); }

static void cmd_memr(const char *arg)
{
    if (!arg||!*arg) { putStr("need address\r\n"); return; }
    {
        uint32_t addr = strtoul(arg,NULL,16);
        if(!addrOK(addr)) { putStr("addr out of range\r\n"); return; }
        {
            uint32_t v = *(volatile uint32_t*)addr;
            putHex32(addr); putStr(" : "); putHex32(v); putStr("\r\n");
        }
    }
}

/* ---- GPIO Command ---- */
static void cmd_gpio(const char *args)
{
    if (!args) { putStr("usage: -gpio idx op ...\r\n"); errorCount[ERR_PARSE_GPIO]++; return; }

    /* Parse index */
    while(*args==' ') args++;      /* skip spaces */
    if(!isdigit((unsigned char)*args)) { putStr("bad idx\r\n"); errorCount[ERR_PARSE_GPIO]++; return; }
    int idx = 0;
    while(isdigit((unsigned char)*args)) { idx = idx*10 + (*args-'0'); args++; }
    if(idx<0 || idx>7) { putStr("idx out of range\r\n"); errorCount[ERR_BAD_GPIO]++; return; }

    while(*args==' ') args++;      /* op char */
    char op = *args;
    if(op!='r' && op!='w' && op!='t') { putStr("op must be r/w/t\r\n"); errorCount[ERR_PARSE_GPIO]++; return; }
    args++;

    uint_least8_t pin = gpioMap[idx];

    if(op=='r') {
        int val = GPIO_read(pin);
        putStr("GPIO "); putDec(idx); putStr(" = "); putDec((unsigned)val); putStr("\r\n");
    }
    else if(op=='w') {
        while(*args==' ') args++;
        if(*args!='0' && *args!='1') { putStr("need 0 or 1\r\n"); errorCount[ERR_PARSE_GPIO]++; return; }
        uint32_t v = (*args=='1');
        GPIO_write(pin, v);
    }
    else /* op=='t' */ {
        if(idx>=6) { putStr("cannot toggle input\r\n"); errorCount[ERR_BAD_GPIO]++; return; }
        GPIO_toggle(pin);
    }
}

/*   PARSER / DISPATCH */
static void handleLine(char *line)
{
    char *cmd = strtok(line," \t");
    char *args = strtok(NULL,"");
    if(!cmd) return;

    if(*cmd!='-') { errorCount[ERR_UNKNOWN_CMD]++; putStr("?? unknown (expected leading '-')\r\n"); return; }
    cmd++;
    if(args && *args=='-') args++;

    if      (!strcmp(cmd,"help"))  cmd_help(args);
    else if (!strcmp(cmd,"about")) cmd_about();
    else if (!strcmp(cmd,"print")) cmd_print(args);
    else if (!strcmp(cmd,"memr"))  cmd_memr(args);
    else if (!strcmp(cmd,"gpio"))  cmd_gpio(args);
    else if (!strcmp(cmd,"error")) cmd_error();
    else {
        errorCount[ERR_UNKNOWN_CMD]++; putStr("?? unknown command\r\n");
    }
}

/*   EDITING HELPERS  (unchanged) */
static void redrawLine(size_t oldLen)
{
    putStr("\r"); prompt(); UART_write(gUart,gLineBuf,len);
    {
        size_t clear=2+oldLen, i; for(i=0;i<clear;i++) putChar(' ');
    }
    putStr("\r"); prompt(); if(cursor) UART_write(gUart,gLineBuf,cursor);
}
static void deleteAtCursor(void)
{
    if(cursor==0) return; {
        size_t oldLen=len, i; for(i=cursor-1;i<len-1;i++) gLineBuf[i]=gLineBuf[i+1]; len--; cursor--; redrawLine(oldLen);
    }
}
static void killLine(void) { size_t old=len; len=cursor=0; redrawLine(old); }

/*   MAIN SHELL TASK  */
void *mainThread(void *arg0)
{
    GPIO_init(); UART_init();

    /* Initial LED and extra GPIO states */
    GPIO_write(CONFIG_GPIO_LED_0,0);
    GPIO_write(CONFIG_GPIO_LED_1,0);
    GPIO_write(CONFIG_GPIO_LED_2,0);
    GPIO_write(CONFIG_GPIO_LED_3,0);
    GPIO_write(CONFIG_GPIO_PK5,0); /* idx4 LOW */
    GPIO_write(CONFIG_GPIO_PD4,1); /* idx5 HIGH */

    UART_Params p; UART_Params_init(&p);
    p.baudRate=115200; p.readDataMode=UART_DATA_BINARY; p.writeDataMode=UART_DATA_BINARY; p.readReturnMode=UART_RETURN_FULL;
    gUart = UART_open(CONFIG_UART_0,&p); if(!gUart) while(1);

    banner();

    {
        char ch;
        for(;;) {
            UART_read(gUart,&ch,1);
            if(ch=='\r'||ch=='\n') {
                putStr("\r\n"); if(len){ strncpy(history,gLineBuf,len); history[len]='\0'; hasHistory=true; gLineBuf[len]='\0'; handleLine(gLineBuf);} len=cursor=0; prompt(); continue; }
            if(ch==0x08||ch==0x7F){ deleteAtCursor(); continue; }
            if(ch==0x15){ killLine(); continue; }
            if(ch==0x1B){ char s1,s2; UART_read(gUart,&s1,1); UART_read(gUart,&s2,1); if(s1=='['){ if(s2=='A'){ if(hasHistory){ size_t old=len; strcpy(gLineBuf,history); len=cursor=strlen(history); redrawLine(old);} } else if(s2=='B'){ size_t old=len; len=cursor=0; redrawLine(old);} else if(s2=='C'){ if(cursor<len){ putChar(gLineBuf[cursor]); cursor++;}} else if(s2=='D'){ if(cursor>0){ putStr("\b"); cursor--;}} } continue; }
            if(isprint((unsigned char)ch)) {
                if(len<MAX_CMD_LEN-1){ if(cursor<len){ size_t old=len,i; for(i=len;i>cursor;i--) gLineBuf[i]=gLineBuf[i-1]; gLineBuf[cursor]=ch; len++; cursor++; redrawLine(old);} else { gLineBuf[len++]=ch; cursor=len; putChar(ch);} }
                else { putStr("\r\n!! character-overflow (31 max) â€“ start again\r\n"); errorCount[ERR_OVERFLOW]++; len=cursor=0; prompt(); }
            }
        }
    }
}
