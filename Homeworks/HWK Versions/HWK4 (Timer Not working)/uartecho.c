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
 *     Code remains C89 â€˜compliant (no mixed decls) for TI toolchain.
 *
 *  NOTE: Ensure SysConfig (ti_drivers_config.h) defines the following indices
 *        or edit gpioMap[] accordingly:
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
#include <ti/drivers/Timer.h>
#include "ti_drivers_config.h"

/* FORWARD DECLARATIONS (C89 COMPLIANT) */
static void putStr(const char *s);
static void putChar(char c);
static void putHex32(uint32_t v);
static void putDec(unsigned v);
static void prompt(void);
static void banner(void);
static void cmd_about(void);
static void cmd_error(void);
static void help_detail(const char *topic);
static void help_ticker(void);
static void cmd_help(const char *args);
static void cmd_print(const char *text);
static bool addrOK(uint32_t a);
static void cmd_memr(const char *arg);
static void cmd_gpio(const char *args);
static int parse_time(const char *arg, uint32_t *out_us);
static void cmd_timer(const char *args);
static void cmd_callback(const char *args);
static void cmd_ticker(const char *args);
static void exec_callback(int idx);
static void print_callback_info(int idx);
static void print_all_callbacks(void);
static void print_ticker_info(int idx);
static void print_all_tickers(void);
static void handleLine(char *line);
static void redrawLine(size_t oldLen);
static void deleteAtCursor(void);
static void killLine(void);
void timerCb(Timer_Handle handle, int_fast16_t status);
void tickerTimerCb(Timer_Handle handle, int_fast16_t status);
void sw1Cb(uint_least8_t index);
void sw2Cb(uint_least8_t index);

/* ---- CALLBACK EVENT QUEUE ---- */
#define CBQ_SZ 8
#define CBQ_TICKER_BASE   1000   // New: Ticker event base

volatile int cbq[CBQ_SZ];
volatile unsigned cbq_head = 0, cbq_tail = 0;

static int cbq_pop(void)
{
    int val = -1;
    if (cbq_head != cbq_tail) {
        val = cbq[cbq_tail];
        cbq_tail = (cbq_tail + 1) % CBQ_SZ;
    }
    return val;
}
static void cbq_push(int idx)
{
    unsigned next = (cbq_head + 1) % CBQ_SZ;
    if (next != cbq_tail) {
        cbq[cbq_head] = idx;
        cbq_head = next;
    }
}

/* METADATA */
#define ABOUT_NAME        "Salim Sadman Bishal"
#define ABOUT_ASSIGNMENT  "ECE 5380 HWK"
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

/* CALLBACK & TIMER SYSTEM (NEW) */
#define MAX_CALLBACKS 3
typedef struct {
    bool active;
    int  count;        /* <0 means infinite */
    char payload[48];  /* shell command payload */
} CallbackEntry;

// Callback 0: timer; 1: SW1 (GPIO 6); 2: SW2 (GPIO 7)
static CallbackEntry callbacks[MAX_CALLBACKS] = {0};

// Timer period, in microseconds (default: 1s)
static uint32_t timer_us = 1000000;
static Timer_Handle gTimer = NULL;

/* ---- TICKER SYSTEM ---- */
#define MAX_TICKERS 16
typedef struct {
    bool active;
    int  count;           // <0 = infinite
    uint32_t delay;       // ticks to wait before first fire
    uint32_t period;      // ticks between repeats
    uint32_t ticks_left;  // countdown (initially = delay, then = period)
    char payload[48];     // scheduled shell command
} TickerEntry;

static TickerEntry tickers[MAX_TICKERS] = {0};
static Timer_Handle gTickerTimer = NULL; // for 10ms periodic ticks

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
    char msg[192];
    snprintf(msg,sizeof(msg),"%s | %s | %s | built %s %s \r\n",ABOUT_NAME,ABOUT_ASSIGNMENT,APP_VERSION,BUILD_DATE,BUILD_TIME);
    putStr(msg);
}

static void cmd_error(void)
{
    putStr("Errors:\r\n  unknown_cmd : "); putDec(errorCount[ERR_UNKNOWN_CMD]); putStr("\r\n");
    putStr("  overflow    : "); putDec(errorCount[ERR_OVERFLOW]);    putStr("\r\n");
    putStr("  bad_gpio    : "); putDec(errorCount[ERR_BAD_GPIO]);    putStr("\r\n");
    putStr("  parse_gpio  : "); putDec(errorCount[ERR_PARSE_GPIO]);  putStr("\r\n");
}

static void help_ticker(void) {
    putStr("-ticker : show all tickers\r\n"
           "-ticker clear idx : clear ticker idx (0-15)\r\n"
           "-ticker idx delay period count -payload : schedule a command\r\n"
           "  idx: ticker index (0-15)\r\n"
           "  delay: ticks to wait (10 ms each) before first fire\r\n"
           "  period: ticks between repeats (10 ms each)\r\n"
           "  count: times to repeat (<0 = forever)\r\n"
           "  payload: shell command to run (any valid shell command)\r\n"
           "Examples:\r\n"
           "  -ticker 3 100 100 5 -gpio 2 t\r\n"
           "    (idx 3, 1s initial delay, 1s between repeats, 5 repeats, toggles gpio 2)\r\n"
           "  -ticker 2 10 50 -1 -print Hello\r\n"
           "    (idx 2, 100ms initial, every 500ms forever, prints Hello)\r\n"
           "  -ticker clear 3\r\n"
           "    (turns off ticker 3)\r\n"
           );
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
    } else if (!strcmp(topic,"timer")) {
        putStr("-timer         : print current timer 0 period (us)\r\n"
               "-timer 0       : turn timer 0 off\r\n"
               "-timer val     : set timer 0 period (us)\r\n"
               "-timer val m   : set timer 0 period (ms)\r\n"
               "-timer val s   : set timer 0 period (s)\r\n"
               "Example: -timer 1000 m  (sets 1s period)\r\n");
    } else if (!strcmp(topic,"callback")) {
        putStr("-callback           : show all callback info\r\n"
                "-callback idx count -payload : set callback idx (0-2), count (<0=forever), and payload\r\n"
                "  idx 0: timer, 1: SW1, 2: SW2\r\n"
                "  count: number of triggers, <0 infinite\r\n"
                "  payload: e.g. -print hello, -gpio 2 t, etc\r\n"
                "-callback clear idx : clear (disable) callback idx\r\n"
                "Example: -callback 1 2 -gpio 3 t\r\n");
    } else if (!strcmp(topic,"ticker")) {
        help_ticker();
    } else {
        putStr("No help for that topic\r\n");
    }
}

static void cmd_help(const char *args)
{
    if (!args || *args=='\0') {
        putStr("Commands: -help  -about  -print  -memr  -gpio  -timer  -callback  -ticker  -error\r\nUse -help <cmd> for details.\r\n");
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

/* ---- TIMER/CALLBACK COMMANDS ---- */
static int parse_time(const char *arg, uint32_t *out_us) {
    if (!arg || !*arg) return -1;
    char *end;
    unsigned long v = strtoul(arg, &end, 10);
    if (v == 0 && arg == end) return -1;
    uint32_t mult = 1; /* default: us */
    while (*end == ' ') ++end;
    if (*end == 'm') mult = 1000;
    else if (*end == 's') mult = 1000000;
    else if (*end && *end != '\0') return -1;
    *out_us = v * mult;
    return 0;
}

static void cmd_timer(const char *args) {
    if (!args || !*args) {
        putStr("Current timer 0 period: "); putDec(timer_us); putStr(" us\r\n");
        return;
    }
    while (*args == ' ') ++args;
    if (!strcmp(args, "0")) {
        if (gTimer) Timer_stop(gTimer);
        putStr("Timer 0 OFF\r\n");
        return;
    }
    {
        uint32_t us;
        if (parse_time(args, &us) == 0) {
            timer_us = us;
            if (gTimer) {
                Timer_stop(gTimer);
                Timer_setPeriod(gTimer, timer_us, Timer_PERIOD_US);
                Timer_start(gTimer);
            }
            putStr("Timer 0 set to "); putDec(timer_us); putStr(" us\r\n");
            return;
        }
    }
    putStr("usage: -timer [0|val|val m|val s]\r\n");
}

/* ---- TICKER COMMAND ---- */
static void print_ticker_info(int idx) {
    putStr("ticker "); putDec(idx); putStr(": ");
    if (!tickers[idx].active) { putStr("off\r\n"); return; }
    putStr("active, delay="); putDec(tickers[idx].delay);
    putStr(", period="); putDec(tickers[idx].period);
    putStr(", count=");
    putDec(tickers[idx].count);
    putStr(", next in "); putDec(tickers[idx].ticks_left); putStr(" ticks");
    if (tickers[idx].payload[0]) { putStr(" -"); putStr(tickers[idx].payload); }
    putStr("\r\n");
}

static void print_all_tickers(void) {
    int i;
    for (i = 0; i < MAX_TICKERS; ++i) print_ticker_info(i);
}

static void cmd_ticker(const char *args) {
    while (args && *args == ' ') ++args;
    if (!args || !*args) {
        putStr("Ticker info\r\n");
        print_all_tickers();
        return;
    }
    // Clear: -ticker clear idx
    if (!strncmp(args, "clear", 5)) {
        int idx = atoi(args + 5);
        if (idx >= 0 && idx < MAX_TICKERS) {
            tickers[idx].active = false;
            tickers[idx].payload[0] = '\0';
            putStr("Cleared ticker "); putDec(idx); putStr("\r\n");
        }
        return;
    }
    // Format: -ticker idx delay period count -payload
    int idx = atoi(args);
    if (idx < 0 || idx >= MAX_TICKERS) {
        putStr("ticker idx 0-15\r\n"); return;
    }
    while (isdigit(*args)) ++args;
    while (*args == ' ') ++args;
    int delay = atoi(args);
    while (isdigit(*args)) ++args;
    while (*args == ' ') ++args;
    int period = atoi(args);
    while (isdigit(*args)) ++args;
    while (*args == ' ') ++args;
    int count = atoi(args);
    while (isdigit(*args) || *args == '-') ++args;
    while (*args == ' ') ++args;
    if (!*args || *args != '-') { putStr("usage: -ticker idx delay period count -cmd ...\r\n"); return; }
    // Set ticker entry
    tickers[idx].active = true;
    tickers[idx].delay = delay;
    tickers[idx].period = period;
    tickers[idx].count = count;
    tickers[idx].ticks_left = delay;
    strncpy(tickers[idx].payload, args + 1, sizeof(tickers[idx].payload) - 1);
    print_ticker_info(idx);
}

/* ---- CALLBACK COMMANDS ---- */
static void print_callback_info(int idx) {
    static const char *names[MAX_CALLBACKS] = {
        "timer", "SW1 Right", "SW2 Left"
    };
    putStr("callback "); putDec(idx); putStr(" is ");
    putStr(names[idx]);
    putStr(", count is ");
    if (!callbacks[idx].active)
        putStr("off");
    else
        putDec(callbacks[idx].count);
    if (callbacks[idx].payload[0]) {
        putStr(" -");
        putStr(callbacks[idx].payload);
    }
    putStr("\r\n");
}
static void print_all_callbacks(void) {
    int i;
    for (i = 0; i < MAX_CALLBACKS; ++i) print_callback_info(i);
}

static void cmd_callback(const char *args) {
    while (args && *args == ' ') ++args;
    if (!args || !*args) {
        putStr("Callback info\r\n");
        print_all_callbacks();
        return;
    }
    /* clear callback */
    if (!strncmp(args, "clear", 5)) {
        int idx = atoi(args + 5);
        if (idx >= 0 && idx < MAX_CALLBACKS) {
            callbacks[idx].active = false;
            callbacks[idx].payload[0] = '\0';
            putStr("Cleared callback "); putDec(idx); putStr("\r\n");
        }
        return;
    }
    /* parse: idx count payload */
    {
        int idx = atoi(args);
        if (idx < 0 || idx >= MAX_CALLBACKS) {
            putStr("callback idx 0-2\r\n"); return;
        }
        while (isdigit(*args)) ++args;
        while (*args == ' ') ++args;
        int count = atoi(args);
        while (isdigit(*args) || *args == '-') ++args;
        while (*args == ' ') ++args;
        if (!*args || *args != '-') { putStr("usage: -callback idx count -cmd ...\r\n"); return; }
        /* Store entry */
        callbacks[idx].active = true;
        callbacks[idx].count = count;
        strncpy(callbacks[idx].payload, args+1, sizeof(callbacks[idx].payload)-1);

        print_callback_info(idx);
    }
}

/* ---- CALLBACK EXECUTION LOGIC ---- */
static void exec_callback(int idx) {
    if (!callbacks[idx].active) return;
    {
        char cmdline[64] = {0};
        strncpy(cmdline, callbacks[idx].payload, sizeof(cmdline)-1);
        if (cmdline[0]) handleLine(cmdline);
    }
    /* Decrement and deactivate if needed */
    if (callbacks[idx].count > 0) {
        callbacks[idx].count--;
        if (callbacks[idx].count == 0) callbacks[idx].active = false;
    }
}

/* ---- TICKER TIMER CALLBACK ---- */
void tickerTimerCb(Timer_Handle handle, int_fast16_t status) {
    putStr("[tickerTimerCb]\r\n");
    int i;
    for (i = 0; i < MAX_TICKERS; ++i) {
        if (!tickers[i].active) continue;
        if (tickers[i].ticks_left > 0) {
            tickers[i].ticks_left--;
        }
        if (tickers[i].ticks_left == 0) {
            // Fire the ticker payload by queuing an event for main loop
            cbq_push(CBQ_TICKER_BASE + i);

            // Update repeat count and reschedule or deactivate
            if (tickers[i].count > 0) {
                tickers[i].count--;
            }
            if (tickers[i].count == 0) {
                tickers[i].active = false;
            } else {
                tickers[i].ticks_left = tickers[i].period;
            }
        }
    }
}

/* ---- CALLBACK TIMER CALLBACKS ---- */
void timerCb(Timer_Handle handle, int_fast16_t status) {
    putStr("[timerCb]\r\n");
    if (callbacks[0].active) cbq_push(0);
}
void sw1Cb(uint_least8_t index) {
    if (callbacks[1].active) cbq_push(1);
}
void sw2Cb(uint_least8_t index) {
    if (callbacks[2].active) cbq_push(2);
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
    else if (!strcmp(cmd,"timer"))   cmd_timer(args);
    else if (!strcmp(cmd,"callback"))cmd_callback(args);
    else if (!strcmp(cmd,"ticker")) cmd_ticker(args);
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
        size_t clear=2+oldLen;
        size_t i;
        for(i=0;i<clear;i++) putChar(' ');
    }
    putStr("\r"); prompt(); if(cursor) UART_write(gUart,gLineBuf,cursor);
}
static void deleteAtCursor(void)
{
    if(cursor==0) return;
    {
        size_t oldLen=len;
        size_t i;
        for(i=cursor-1;i<len-1;i++) gLineBuf[i]=gLineBuf[i+1];
        len--; cursor--; redrawLine(oldLen);
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

    /* Timer setup */
    {
        Timer_Params tp; Timer_Params_init(&tp);
        tp.periodUnits = Timer_PERIOD_US;
        tp.period = timer_us;
        tp.timerMode = Timer_CONTINUOUS_CALLBACK;
        tp.timerCallback = timerCb;
        gTimer = Timer_open(Timer0, &tp);
        if (!gTimer) putStr("ERROR: Timer0 failed to open!\r\n");
    }
    /* Ticker timer setup (10ms) */
    {
        Timer_Params ttp; Timer_Params_init(&ttp);
        ttp.periodUnits = Timer_PERIOD_US;
        ttp.period = 10000; // 10 ms
        ttp.timerMode = Timer_CONTINUOUS_CALLBACK;
        ttp.timerCallback = tickerTimerCb;
        gTickerTimer = Timer_open(Timer1, &ttp);
        if (!gTickerTimer) putStr("ERROR: Timer1 failed to open!\r\n");
    }

    /* Register GPIO interrupts for switches (SW1, SW2) */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, sw1Cb);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, sw2Cb);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    banner();

    {
        char ch;
        for(;;) {
            UART_read(gUart,&ch,1);

            /* Process pending callback and ticker events */
            {
                int cbidx;
                while ((cbidx = cbq_pop()) != -1) {
                    if (cbidx >= CBQ_TICKER_BASE && cbidx < CBQ_TICKER_BASE + MAX_TICKERS) {
                        int tidx = cbidx - CBQ_TICKER_BASE;
                        char cmdline[64] = {0};
                        strncpy(cmdline, tickers[tidx].payload, sizeof(cmdline)-1);
                        if (cmdline[0]) handleLine(cmdline);
                    } else {
                        exec_callback(cbidx);
                    }
                }
            }

            if(ch=='\r'||ch=='\n') {
                putStr("\r\n");
                if(len){
                    strncpy(history,gLineBuf,len);
                    history[len]='\0';
                    hasHistory=true;
                    gLineBuf[len]='\0';
                    handleLine(gLineBuf);
                }
                len=cursor=0;
                prompt();
                continue;
            }
            if(ch==0x08||ch==0x7F){ deleteAtCursor(); continue; }
            if(ch==0x15){ killLine(); continue; }
            if(ch==0x1B){
                char s1,s2;
                UART_read(gUart,&s1,1); UART_read(gUart,&s2,1);
                if(s1=='['){
                    if(s2=='A'){
                        if(hasHistory){
                            size_t old=len;
                            strcpy(gLineBuf,history);
                            len=cursor=strlen(history);
                            redrawLine(old);
                        }
                    }
                    else if(s2=='B'){
                        size_t old=len; len=cursor=0; redrawLine(old);
                    }
                    else if(s2=='C'){
                        if(cursor<len){ putChar(gLineBuf[cursor]); cursor++;}
                    }
                    else if(s2=='D'){
                        if(cursor>0){ putStr("\b"); cursor--;}
                    }
                }
                continue;
            }
            if(isprint((unsigned char)ch)) {
                if(len<MAX_CMD_LEN-1){
                    if(cursor<len){
                        size_t old=len,i;
                        for(i=len;i>cursor;i--) gLineBuf[i]=gLineBuf[i-1];
                        gLineBuf[cursor]=ch; len++; cursor++; redrawLine(old);
                    }
                    else { gLineBuf[len++]=ch; cursor=len; putChar(ch);}
                }
                else {
                    putStr("\r\n!! character-overflow (31 max) start again\r\n");
                    errorCount[ERR_OVERFLOW]++; len=cursor=0; prompt();
                }
            }
        }
    }
}
