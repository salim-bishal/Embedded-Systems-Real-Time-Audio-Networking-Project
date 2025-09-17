/*
 *  MSP432E401Y Enhanced Command-Line Shell v2.0.2
 *  Adds: callback and ticker with flag polling. Safe, robust, and simple.
 *  Author: Salim Sadman Bishal
 *  (based on reference code/logic from working projects)
 *
 *  SysConfig requirements:
 *    GPIO  : CONFIG_GPIO_LED_0..3, CONFIG_GPIO_PK5, CONFIG_GPIO_PD4,
 *            CONFIG_GPIO_BUTTON_0, CONFIG_GPIO_BUTTON_1
 *    UART  : CONFIG_UART_0
 *    Timer : CONFIG_TIMER_0, CONFIG_TIMER_1
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>
#include "ti_drivers_config.h"

#ifndef CONFIG_TIMER_0
#error "Add a Timer peripheral named CONFIG_TIMER_0 in SysConfig"
#endif
#ifndef CONFIG_TIMER_1
#error "Add a Timer peripheral named CONFIG_TIMER_1 in SysConfig"
#endif

#define ABOUT_NAME   "Salim Sadman Bishal"
#define ABOUT_ASSIGNMENT "ECE 5380 HWKs"
#define APP_VERSION  "v2.0.5"
#define BUILD_DATE        __DATE__
#define BUILD_TIME        __TIME__

#define RX_BUF_SZ     128
#define MAX_CMD_LEN   128
#define MAX_PAYLOAD   64

#define NUM_REGISTERS 32
#define SCRIPT_LINES 64
#define SCRIPT_LINE_SIZE 128

static int32_t registers[NUM_REGISTERS] = {0};
static char scriptLines[SCRIPT_LINES][SCRIPT_LINE_SIZE] = {{0}};


/* index â†’ GPIO mapping */
static const uint_least8_t gpioMap[8] = {
    CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_3,
    CONFIG_GPIO_PK5,   CONFIG_GPIO_PD4,
    CONFIG_GPIO_BUTTON_0, CONFIG_GPIO_BUTTON_1
};

/* error counters */
enum { ERR_UNKNOWN_CMD, ERR_OVERFLOW, ERR_BAD_GPIO, ERR_PARSE_GPIO, NUM_ERR };
static unsigned errorCount[NUM_ERR] = {0};

/* runtime state */
static UART_Handle  gUart;
static Timer_Handle gSysTimer = NULL;     // for callback (CONFIG_TIMER_0)
static Timer_Handle gTickerTimer = NULL;  // for ticker   (CONFIG_TIMER_1)
static unsigned     currentPeriodUs = 0;

/* line-editing */
static char   gLineBuf[RX_BUF_SZ];
static size_t len = 0, cursor = 0;
static char   history[RX_BUF_SZ];
static bool   hasHistory = false;

/* ========== Callback and Ticker ========== */
#define MAX_CB 3
static const char *cb_names[MAX_CB] = { "timer", "SW1", "SW2" };
#define MAX_TICKERS 16
#define MAX_TICKER_PAYLOAD 48


struct CbEntry {
    bool active;
    int32_t remaining;
    char payload[MAX_PAYLOAD];
} cb[MAX_CB];
/* ---- Utility I/O helpers ---- */
static void putStr(const char*s){ UART_write(gUart,s,strlen(s)); }
static void putChar(char c){ UART_write(gUart,&c,1); }
static void putDec(int v){ char b[12]; snprintf(b,12,"%d",v); putStr(b); }
static void prompt(void){ putStr("> "); }
static void banner(void)
{
    putStr("\r\n*** MSP432 Command Shell Ready ***\r\n");
    putStr("Type -help for a list of commands.\r\n\r\n");
    prompt();
}

static void print_all_callbacks(void) {
    int i;
    for(i=0; i<MAX_CB; ++i) {
        putStr("callback "); putDec(i); putStr(" is ");
        putStr(cb_names[i]); putStr(", count is ");
        if (!cb[i].active)
            putStr("off");
        else
            putDec(cb[i].remaining);
        if (cb[i].payload[0]) {
            putStr(" -");
            putStr(cb[i].payload);
        }
        putStr("\r\n");
    }
}


struct TickerEntry {
    bool active;
    uint32_t delay_ticks;         // initial delay in 10ms ticks
    uint32_t period_ticks;        // period in 10ms ticks
    int32_t  count;           // repeat count (<0 for infinite)
    char     payload[MAX_TICKER_PAYLOAD];
    uint32_t ticks_left;          // current countdown
} ticker[MAX_TICKERS] = {0};
static void print_all_tickers(void) {
    int i;
    putStr("Idx | Active | Delay | Period | Count | Payload\r\n");
    for(i=0; i<MAX_TICKERS; ++i) {
        putDec(i); putStr("   | ");
        putStr(ticker[i].active ? " Yes  | " : "  No  | ");
        putDec(ticker[i].delay_ticks); putStr("    | ");
        putDec(ticker[i].period_ticks); putStr("     | ");
        putDec(ticker[i].count); putStr("     | ");
        putStr(ticker[i].payload); putStr("\r\n");
    }
}

/* ---- Event flags ---- */
static volatile bool tickFlag = false;    // for callback timer (1s, or as set)
static volatile bool tickerFlag = false;  // for 10ms ticker
static volatile bool sw1Flag = false;
static volatile bool sw2Flag = false;

/* ---- ISRs ---- */
static void timerIsr(Timer_Handle h, int_fast16_t id)   { (void)h; (void)id; tickFlag = true; }
static void tickerIsr(Timer_Handle h, int_fast16_t id)  { (void)h; (void)id; tickerFlag = true; }
static void sw1Isr(uint_least8_t i)                     { (void)i; sw1Flag = true; }
static void sw2Isr(uint_least8_t i)                     { (void)i; sw2Flag = true; }



/* forward decl for nested execution */
static void handleLine(char*);

/* execute payload string */
static void execPayload(const char* p) {
    if(p && *p && strlen(p) < RX_BUF_SZ) {
        char payloadBuf[RX_BUF_SZ];
        strcpy(payloadBuf, p);
        handleLine(payloadBuf);
    }
}

/* ---- command prototypes ---- */
static void cmd_help(const char*);
static void cmd_about(void);
static void cmd_gpio(const char*);
static void cmd_timer(const char*);
static void cmd_callback(const char*);
static void cmd_ticker(const char*);
static void cmd_error(void);
static void cmd_print(const char*);
static void cmd_memr(const char*);
static void cmd_rem(const char*);
static void cmd_if(const char*);



static int parse_register(const char *token) {
    if (!token) return -1;
    if ((token[0] == 'r' || token[0] == 'R')) {
        int n = atoi(&token[1]);
        if (n >= 0 && n < NUM_REGISTERS) return n;
    }
    return -1;
}
static bool parse_immediate(const char *token, int32_t *value) {
    if (!token || token[0] != '#') return false;
    char *endptr;
    if (token[1] == 'x' || token[1] == 'X') {
        *value = strtol(&token[2], &endptr, 16);
    } else {
        *value = strtol(&token[1], &endptr, 10);
    }
    return (*endptr == '\0');
}

// Allow @address (direct) and @R<n> (indirect via register content)
static bool parse_memory_address(const char *token, uint32_t *addr) {
    if(!token || token[0]!='@') return false;
    if(token[1]=='r'||token[1]=='R') {
        int r=parse_register(&token[1]);
        if(r>=0) { *addr=(uint32_t)registers[r]; return true; }
    } else if(token[1]=='x'||token[1]=='X') {
        *addr = strtoul(&token[2],NULL,16); return true;
    } else {
        *addr = strtoul(&token[1],NULL,10); return true;
    }
    return false;
}

static bool get_operand_value(const char *token, int32_t *out) {
    int r = parse_register(token);
    if(r >= 0) { *out = registers[r]; return true; }
    if(parse_immediate(token, out)) return true;
    uint32_t addr;
    if(parse_memory_address(token, &addr)) { *out = *(int32_t*)addr; return true; }
    return false;
}


static void cmd_reg(const char *args) {
    int i;
    if (!args || !*args) {
        char buf[64];
        putStr("R  Value\n--------\n");
        for(i=0; i<NUM_REGISTERS; ++i) {
            snprintf(buf, sizeof(buf), "R%d = %ld\r\n", i, (long)registers[i]);
            putStr(buf);
        }
        return;
    }
    char op[8]={0}, tok1[16]={0}, tok2[16]={0};
    int n = sscanf(args, "%7s %15s %15s", op, tok1, tok2);
    if (n < 2) { putStr("Usage: -reg OP DST [SRC]\r\n"); return; }
    int dreg = parse_register(tok1);
    int32_t val = 0;
    if (!strcasecmp(op,"mov")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] = val;
    }
    else if (!strcasecmp(op,"xchg")) {
        int sreg = parse_register(tok2);
        if (dreg < 0 || sreg < 0) { putStr("Bad reg\r\n"); return; }
        int32_t tmp = registers[dreg]; registers[dreg] = registers[sreg]; registers[sreg] = tmp;
    }
    else if (!strcasecmp(op,"inc")) {
        if (dreg < 0) { putStr("Bad reg\r\n"); return; } registers[dreg]++;
    }
    else if (!strcasecmp(op,"dec")) {
        if (dreg < 0) { putStr("Bad reg\r\n"); return; } registers[dreg]--;
    }
    else if (!strcasecmp(op,"neg")) {
        if (dreg < 0) { putStr("Bad reg\r\n"); return; } registers[dreg] = -registers[dreg];
    }
    else if (!strcasecmp(op,"not")) {
        if (dreg < 0) { putStr("Bad reg\r\n"); return; } registers[dreg] = ~registers[dreg];
    }
    else if (!strcasecmp(op,"add")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] += val;
    }
    else if (!strcasecmp(op,"sub")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] -= val;
    }
    else if (!strcasecmp(op,"mul")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] *= val;
    }
    else if (!strcasecmp(op,"div")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        if (val == 0) { putStr("div0\r\n"); return; }
        registers[dreg] /= val;
    }
    else if (!strcasecmp(op,"rem")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        if (val == 0) { putStr("div0\r\n"); return; }
        registers[dreg] %= val;
    }
    else if (!strcasecmp(op,"and")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] &= val;
    }
    else if (!strcasecmp(op,"ior")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] |= val;
    }
    else if (!strcasecmp(op,"xor")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        registers[dreg] ^= val;
    }
    else if (!strcasecmp(op,"max")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        if (registers[dreg] < val) registers[dreg] = val;
    }
    else if (!strcasecmp(op,"min")) {
        if (!get_operand_value(tok2, &val)) { putStr("Bad src\r\n"); return; }
        if (dreg < 0) { putStr("Bad dst\r\n"); return; }
        if (registers[dreg] > val) registers[dreg] = val;
    }
    else { putStr("Bad op\r\n"); return; }
    if (dreg >= 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "R%d=%ld\r\n", dreg, (long)registers[dreg]);
        putStr(buf);
    }
}

static bool get_if_operand(const char *tok, int32_t *out) {
    if(!tok) return false;
    if(tok[0]=='#') {
        char *endptr;
        *out = strtol(&tok[1], &endptr, 0);
        return (*endptr == '\0');
    } else if(tok[0]=='r' || tok[0]=='R') {
        int n = atoi(&tok[1]);
        if(n >= 0 && n < 32) { *out = registers[n]; return true; }
    }
    return false;
}

static void cmd_if(const char* args) {
    // Format: -if A COND B ? DESTT : DESTF
    char tokA[16]={0}, cond[3]={0}, tokB[16]={0};
    const char *q=args;
    // Parse 3 tokens: tokA, cond, tokB
    int n = sscanf(q, "%15s %2s %15s", tokA, cond, tokB);
    if(n < 3) { putStr("Usage: -if A COND B ? DESTT : DESTF\r\n"); return; }

    // Now scan to find the first '?'
    const char *qmark = strchr(q, '?');
    if(!qmark) { putStr("Bad '?' in -if\r\n"); return; }
    // Extract before ? (already parsed), after ? is actions
    qmark++; // move past '?'
    // Find ':'
    const char *colon = strchr(qmark, ':');
    char destt[64]={0}, destf[64]={0};
    if(colon) {
        // Extract DESTT (between ? and :)
        int len = colon - qmark;
        while(len > 0 && isspace((unsigned char)qmark[len-1])) len--;
        strncpy(destt, qmark, (len > 0 ? len : 0)); destt[len] = 0;
        // Extract DESTF (after :)
        colon++;
        while(*colon && isspace((unsigned char)*colon)) colon++;
        strncpy(destf, colon, 63); destf[63]=0;
    } else {
        // No ':' present, only DESTT
        strncpy(destt, qmark, 63); destt[63]=0;
    }
    // Trim leading spaces
    char *dt = destt; while(*dt && isspace((unsigned char)*dt)) dt++;
    char *df = destf; while(*df && isspace((unsigned char)*df)) df++;

    // Parse operands
    int32_t a=0, b=0;
    if(!get_if_operand(tokA, &a)) { putStr("Bad A\r\n"); return; }
    if(!get_if_operand(tokB, &b)) { putStr("Bad B\r\n"); return; }
    bool res = false;
    if(cond[0]=='>') res = (a > b);
    else if(cond[0]=='<') res = (a < b);
    else if(cond[0]=='=') res = (a == b);
    else { putStr("COND?\r\n"); return; }

    if(res) {
        if(*dt) handleLine(dt);
    } else {
        if(*df) handleLine(df);
    }
}


// --------- SCRIPT HANDLER ----------
static void print_all_script_lines(void) {
    char buf[80];
    putStr("Line | Script Line\n------------------------------\n");
    int i;
    for (i = 0; i < SCRIPT_LINES; ++i) {
        snprintf(buf, sizeof(buf), "%2d   | %s\r\n", i, scriptLines[i][0] ? scriptLines[i] : "<empty>");
        putStr(buf);
    }
}
static void print_script_line(int line) {
    if (line < 0 || line >= SCRIPT_LINES) { putStr("Bad line\r\n"); return; }
    char buf[80];
    snprintf(buf, sizeof(buf), "%2d | %s\r\n", line, scriptLines[line][0] ? scriptLines[line] : "<empty>");
    putStr(buf);
}

static void cmd_script(const char* args) {
    if (!args || !*args) { print_all_script_lines(); return; }
    while (*args == ' ') args++;
    int idx = atoi(args);
    while (*args && !isspace((unsigned char)*args)) args++;
    while (*args == ' ') args++;
    if (!*args) { print_script_line(idx); return; }
    if (*args == 'w') { // write
        while (*args && !isspace((unsigned char)*args)) args++;
        while (*args == ' ') args++;
        strncpy(scriptLines[idx], args, SCRIPT_LINE_SIZE-1);
        scriptLines[idx][SCRIPT_LINE_SIZE-1] = '\0';
        putStr("Script "); putDec(idx); putStr(" loaded.\r\n");
        return;
    }
    if (*args == 'x') {
        int i;// execute
        for (i = idx; i < SCRIPT_LINES && scriptLines[i][0]; ++i) {
            char buf[SCRIPT_LINE_SIZE+1];
            strncpy(buf, scriptLines[i], SCRIPT_LINE_SIZE);
            buf[SCRIPT_LINE_SIZE] = '\0';
            handleLine(buf);
        }
        return;
    }
    if (*args == 'c') { // clear
        scriptLines[idx][0] = '\0';
        putStr("Script "); putDec(idx); putStr(" cleared.\r\n");
        return;
    }
    putStr("Usage: -script [line] [w|x|c] [payload]\r\n");
}


/* ---- parser ---- */
static void handleLine(char *line)
{
    char *cmd=strtok(line," \t");
    char *args=strtok(NULL,"");
    if(!cmd) return;
    if(*cmd!='-'){ errorCount[ERR_UNKNOWN_CMD]++; putStr("?? unknown (expected leading '-')\r\n"); return; }
    cmd++;

    if     (!strcmp(cmd,"help"))     cmd_help(args);
    else if(!strcmp(cmd,"about"))    cmd_about();
    else if(!strcmp(cmd,"gpio"))     cmd_gpio(args);
    else if(!strcmp(cmd,"timer"))    cmd_timer(args);
    else if(!strcmp(cmd,"callback")) cmd_callback(args);
    else if(!strcmp(cmd,"ticker"))   cmd_ticker(args);       // Add ticker
    else if(!strcmp(cmd,"error"))    cmd_error();
    else if(!strcmp(cmd,"print"))    cmd_print(args);
    else if(!strcmp(cmd,"memr"))     cmd_memr(args);
    else if(!strcmp(cmd,"reg"))      cmd_reg(args);
    else if(!strcmp(cmd,"script"))   cmd_script(args);
    else if(!strcmp(cmd,"rem"))      cmd_rem(args);
    else if(!strcmp(cmd,"if"))      cmd_if(args);
    else { errorCount[ERR_UNKNOWN_CMD]++; putStr("?? unknown\r\n"); }
}

/* ---- help / about ---- */
static void help_detail(const char*t){
    if (!strcmp(t,"help")) {
        putStr("-help [cmd]   : list all commands or details for <cmd>\r\n");
    } else if (!strcmp(t,"about")) {
        putStr("-about        : show author, assignment, version, build date/time\r\n");
    } else if (!strcmp(t,"print")) {
        putStr("-print text   : echo text exactly as entered\r\n");
    } else if (!strcmp(t,"memr")) {
        putStr("-memr addrhex : read 32-bit word (flash 0x0-0x7FFFF | SRAM 0x20000000-0x2007FFFF)\r\n");
    } else if (!strcmp(t,"gpio")) {
        putStr("-gpio idx op [val]\r\n"
                "  idx 0-3 : LEDs, 4:PK5, 5:PD4, 6-7: switches \r\n"
                "  op  r      : read pin\r\n"
                "      t      : toggle (outputs only)\r\n");
    } else if (!strcmp(t,"error")) {
        putStr("-error       : show error counters since power-up\r\n");
    } else if (!strcmp(t,"timer")) {
        putStr("-timer         : print current timer 0 period (us)\r\n"
                "-timer 0       : turn timer 0 off\r\n"
                "-timer val     : set timer 0 period (us)\r\n"
                "-timer val m   : set timer 0 period (ms)\r\n"
                "-timer val s   : set timer 0 period (s)\r\n"
                "Example: -timer 1000 m  (sets 1s period)\r\n");
    } else if (!strcmp(t,"callback")) {
        putStr("-callback           : show all callback info\r\n"
                "-callback idx count -payload : set callback idx (0-2), count (<0=forever), and payload\r\n"
                "  idx 0: timer, 1: SW1, 2: SW2\r\n"
                "  count: number of triggers, <0 infinite\r\n"
                "  payload: e.g. -print hello, -gpio 2 t, etc\r\n"
                "-callback clear idx : clear (disable) callback idx\r\n"
                "Example: -callback 1 2 -gpio 3 t\r\n");
    } else if (!strcmp(t,"ticker")) {
        putStr("-ticker idx delay period count -payload\r\n");
        putStr("  idx:     0-15 (selects ticker slot)\r\n");
        putStr("  delay:   initial delay, in 10ms ticks before first run\r\n");
        putStr("  period:  repeat interval, in 10ms ticks\r\n");
        putStr("  count:   # of repeats (<0 means infinite)\r\n");
        putStr("  payload: shell command (ex: -gpio 2 t)\r\n");
        putStr("Example:\r\n  -ticker 3 100 100 5 -gpio 2 t\r\n");
        putStr("   (runs ticker #3: after 1s (100x10ms), does 'gpio 2 t' every 1s, 5 times)\r\n");
        putStr("Type -ticker (no args) to see all active tickers and their state.\r\n");
        putStr("-ticker idx 0 (to clear ticker idx)\r\n");
    } else if (!strcmp(t, "reg")){
        putStr("-reg                        : Show all 32 registers and their values\r\n"
               "-reg mov dst src            : Move src value (reg/#imm) to dst register\r\n"
               "-reg xchg rX rY             : Exchange two registers\r\n"
               "-reg inc/dec rX             : Increment/decrement rX\r\n"
                "-reg add/sub/mul/div/rem dst src : dst = dst op src\r\n"
                "-reg not/neg rX             : Bitwise NOT/arith NEG\r\n"
                "-reg and/ior/xor dst src    : Bitwise ops\r\n"
                "-reg max/min dst src        : Maximum/minimum\r\n"
                "Operands: rX, #imm, #xHEX\r\n"
                "Examples:\r\n"
                "  -reg mov r1 #123      (set r1=123)\r\n"
                "  -reg add r2 r1        (r2 += r1)\r\n"
                "  -reg sub r0 #x10      (r0 -= 0x10)\r\n"
                "  -reg xchg r1 r2       (swap r1, r2)\r\n"
                "  -reg inc r3           (r3++)\r\n"
                "  -reg neg r7           (r7 = -r7)\r\n"
                "  -reg mov r2 #xFF      (r2=255)\r\n"
                "  -reg mul r4 #5        (r4 *= 5)\r\n");
    } else if (!strcmp(t, "script")) {
        putStr("-script                  : Display all script lines\r\n"
                "-script N                : Show script line N\r\n"
                "-script N w CMD...       : Write CMD... to script line N\r\n"
                "-script N x              : Execute script from line N\r\n"
                "-script N c              : Clear line N\r\n"
                "Examples:\r\n"
                "  -script 10 w -gpio 0 t        (store toggle LED command at line 10)\r\n"
                "  -script 10 x                  (execute from line 10)\r\n"
                "  -script 10 c                  (clear line 10)\r\n"
                "  -script                       (list all script lines)\r\n");
    } else if (!strcmp(t,"if")) {
        putStr("-if A COND B ? DESTT : DESTF\r\n"
                "  A/B: rN (register) or #IMM\r\n"
                "  COND: >  =  <\r\n"
                "  DESTT: Command if TRUE, DESTF: Command if FALSE\r\n"
                "Example:\r\n"
                "  -if r1 > #0 ? -print OK : -print BAD\r\n"
                "  -if r1 > #0 ? -print OK : -print BAD\r\n"
                "  -if r3 < #10 ? : -print hi\r\n");
    } else if(!strcmp(t,"rem")) {
        putStr("-rem [remark text] : comment line (does nothing)\r\n");
    }
    else {
        putStr("No help for that topic\r\n");
    }
}
static void cmd_help(const char *a) {
    if (!a || !*a) {
        putStr("Commands: -help  -about  -print  -memr  -gpio  -timer  -callback  -ticker -reg -script -rem  -error\r\nUse -help <cmd> for details.\r\n");
    } else {
        if (*a=='-') a++;
        help_detail(a);
    }
}
/* ---- gpio ---- */
static void cmd_gpio(const char*args){
    if(!args || !*args){
        help_detail("gpio");
        return;
    }
    while(*args==' ')args++;
    int idx=atoi(args); while(isdigit((unsigned char)*args))args++;
    if(idx<0||idx>7){ putStr("bad idx\r\n"); return; }
    while(*args==' ')args++;
    char op=*args++;
    uint_least8_t pin=gpioMap[idx];

    if(op=='r'){ putDec(GPIO_read(pin)); putStr("\r\n"); }
    else if(op=='w'){ while(*args==' ')args++; GPIO_write(pin, *args=='1'); }
    else if(op=='t'){ if(idx>=6){ putStr("ro\r\n"); return; } GPIO_toggle(pin); }
}

/* ---- timer ---- */
static void cmd_timer(const char*a){
    if(!a||!*a){
        if(currentPeriodUs==0) putStr("stopped\r\n");
        else { putStr("period "); putDec(currentPeriodUs); putStr(" us\r\n"); }
        return;
    }
    unsigned n=strlen(a);
    char unit = 'u';
    if(a[n-1]=='s'||a[n-1]=='S'){ unit = 's'; n--; }
    else if(a[n-1]=='m' || a[n-1]=='M') { unit = 'm'; n--; }
    char tmp[12]; strncpy(tmp,a,n); tmp[n]='\0';
    long v=strtol(tmp,NULL,10);
    if(v<0){ putStr("bad\r\n"); return; }
    unsigned us = 0;
    if(unit == 's')      us = v * 1000000u;
    else if(unit == 'm') us = v * 1000u;
    else                 us = v;
    if(us==0){ if(gSysTimer)Timer_stop(gSysTimer); currentPeriodUs=0; return; }
    if(!gSysTimer){
        Timer_Params tp; Timer_Params_init(&tp);
        tp.periodUnits=Timer_PERIOD_US;
        tp.timerMode=Timer_CONTINUOUS_CALLBACK; tp.timerCallback=timerIsr;
        gSysTimer=Timer_open(CONFIG_TIMER_0,&tp);
    }
    Timer_setPeriod(gSysTimer,Timer_PERIOD_US,us);
    Timer_start(gSysTimer); currentPeriodUs=us;
}

/* ---- callback ---- */
static void cmd_callback(const char* args) {
    if(!args || !*args){
        print_all_callbacks();
        return;
    }
    while(*args == ' ') args++;
    int idx = atoi(args);
    while(*args && !isspace((unsigned char)*args)) args++;
    if(idx < 0 || idx >= MAX_CB) { putStr("idx0-2\r\n"); return; }
    while(*args == ' ') args++; int cnt = atoi(args);
    while(*args&&*args!=' ')args++; while(*args==' ')args++;
    if(cnt == 0) { cb[idx].active = false; putStr("clr\r\n"); return; }
    cb[idx].active = true; cb[idx].remaining = cnt;
    strncpy(cb[idx].payload, args, MAX_PAYLOAD-1); cb[idx].payload[MAX_PAYLOAD-1] = '\0';
}

/* ---- ticker ---- */
static void cmd_ticker(const char* args) {
    // Usage: -ticker idx delay period count payload
    if(!args || !*args){
        print_all_tickers();
        return; }
    while(*args == ' ') args++;
    int idx = atoi(args); while(isdigit((unsigned char)*args)) args++;
    if(idx < 0 || idx >= MAX_TICKERS) { putStr("idx0-15\r\n"); return; }
    while(*args == ' ') args++; int delay = atoi(args);
    while(*args && !isspace((unsigned char)*args)) args++;

    while(*args == ' ') args++; int period = atoi(args);
    while(*args && !isspace((unsigned char)*args)) args++;

    while(*args == ' ') args++; int cnt = atoi(args);
    while(*args && !isspace((unsigned char)*args)) args++;
    while(*args == ' ') args++;
    if(cnt == 0) { ticker[idx].active = false; putStr("clr\r\n"); return; }
    ticker[idx].active = true;
    ticker[idx].delay_ticks = delay;     // in units of 10ms
    ticker[idx].period_ticks = period;   // in units of 10ms
    ticker[idx].count = cnt;
    strncpy(ticker[idx].payload, args, MAX_TICKER_PAYLOAD-1);
    ticker[idx].payload[MAX_TICKER_PAYLOAD-1] = '\0';
    ticker[idx].ticks_left = delay; // start with initial delay
}

/* ---- error / print / memr ---- */
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



static void cmd_print(const char *text) { if(text) putStr(text); putStr("\r\n"); }

static bool addrOK(uint32_t a)
{ return (a<0x00080000u) || (a>=0x20000000u && a<0x20080000u); }



static void cmd_memr(const char*a){
    if(!a||!*a){ putStr("need addr...\r\n"); return; }
    uint32_t d=strtoul(a,NULL,16);
    if(!addrOK(d)){ putStr("addr our of  range\r\n"); return; }

    char b[11]; snprintf(b,11,"0x%08X",*(volatile uint32_t*)d); putStr(b); putStr("\r\n");
}

static void cmd_rem(const char* args) {
    // Do nothing, just a comment
    (void)args;
}


/* ---- editor helpers ---- */
static void redraw(size_t o){
    putStr("\r"); prompt();
    UART_write(gUart,gLineBuf,len);
    size_t c=2+o,i; for(i=0;i<c;i++) putChar(' ');
    putStr("\r"); prompt();
    if(cursor) UART_write(gUart,gLineBuf,cursor);
}
static void del(void){
    if(cursor==0) return;
    size_t o=len,i; for(i=cursor-1;i<len-1;i++) gLineBuf[i]=gLineBuf[i+1];
    len--; cursor--; redraw(o);
}
static void kill(void){ size_t o=len; len=cursor=0; redraw(o); }

/* ========== MAIN THREAD ========== */
void *mainThread(void *arg0)
{
    GPIO_init(); UART_init(); Timer_init();

    /* buttons */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU|GPIO_CFG_IN_INT_RISING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU|GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0,sw1Isr); GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1,sw2Isr); GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    /* UART (non-blocking) */
    UART_Params p; UART_Params_init(&p);
    p.baudRate=115200;
    p.readDataMode=UART_DATA_BINARY;
    p.writeDataMode=UART_DATA_BINARY;
    p.readReturnMode=UART_RETURN_FULL;
    p.readTimeout   = 1;           /* â†� non-blocking poll */
    gUart = UART_open(CONFIG_UART_0,&p);
    if(!gUart) while(1);

    /* --- Setup callback timer (Timer0) --- */
    Timer_Params tp;
    Timer_Params_init(&tp);
    tp.periodUnits = Timer_PERIOD_US;      // 1ms units for compatibility with ms granularity
    tp.period = 1000*1000;                     // Default: 1s
    tp.timerMode = Timer_CONTINUOUS_CALLBACK;
    tp.timerCallback = timerIsr;
    gSysTimer = Timer_open(CONFIG_TIMER_0, &tp);
    if(gSysTimer) Timer_start(gSysTimer);

    /* --- Setup ticker timer (Timer1) for 10ms granularity --- */
    Timer_Params ttp;
    Timer_Params_init(&ttp);
    ttp.periodUnits = Timer_PERIOD_US;    // 10ms units
    ttp.period = 10*1000;
    ttp.timerMode = Timer_CONTINUOUS_CALLBACK;
    ttp.timerCallback = tickerIsr;
    gTickerTimer = Timer_open(CONFIG_TIMER_1, &ttp);
    if(gTickerTimer) Timer_start(gTickerTimer);

    banner();


    for(;;){
        /* service callbacks */
        if(tickFlag){ tickFlag=false;
            if(cb[0].active){ execPayload(cb[0].payload);
                if(cb[0].remaining>0 && --cb[0].remaining==0) cb[0].active=false; }
        }
        if(sw1Flag){ sw1Flag=false;
            if(cb[1].active){ execPayload(cb[1].payload);
                if(cb[1].remaining>0 && --cb[1].remaining==0) cb[1].active=false; }
        }
        if(sw2Flag){ sw2Flag=false;
            if(cb[2].active){ execPayload(cb[2].payload);
                if(cb[2].remaining>0 && --cb[2].remaining==0) cb[2].active=false; }
        }

        /* poll all active tickers every 10ms */

        if(tickerFlag){ tickerFlag=false;
        int i;
            for(i=0; i<MAX_TICKERS; ++i){
                if(ticker[i].active){
                    if(ticker[i].ticks_left > 0){
                        ticker[i].ticks_left--;
                    }
                    if(ticker[i].ticks_left == 0){
                        execPayload(ticker[i].payload);
                        if(ticker[i].count > 0 && --ticker[i].count == 0){
                            ticker[i].active = false;
                        } else {
                            ticker[i].ticks_left = ticker[i].period_ticks;
                        }
                    }
                }
            }
        }

        /* non-blocking UART poll */
        char ch;
        int n = UART_read(gUart,&ch,1);
        if(n==0) continue;      /* no byte â€” loop again */

        /* line-editing */
        if(ch=='\r'||ch=='\n'){
            putStr("\r\n");
            if(len){
                strncpy(history,gLineBuf,len); history[len]='\0'; hasHistory=true;
                gLineBuf[len]='\0'; handleLine(gLineBuf);
            }
            len=cursor=0; prompt(); continue;
        }
        if(ch==0x08||ch==0x7F){ del(); continue; }
        if(ch==0x15){ kill(); continue; }

        if(ch == 0x1B) { // ESC          /* arrows */
            char s1, s2;
            if(UART_read(gUart,&s1,1)==0)continue;
            if(UART_read(gUart,&s2,1)==0)continue;

            if(s1=='['){
                if(s2=='A'){
                    if(hasHistory){
                        size_t o=len;
                        strcpy(gLineBuf,history);
                        len=cursor=strlen(history);
                        redraw(o);
                    }
                }
                else if(s2=='B'){
                    size_t o=len;
                    len=cursor=0;
                    redraw(o); }
                else if(s2=='C'){
                    if(cursor<len){
                        putChar(gLineBuf[cursor]);
                        cursor++;
                    }
                }
                else if(s2=='D'){
                    if(cursor>0)
                    {putStr("\b");
                    cursor--;
                    }
                }
            }
            continue;
        }

        if(isprint((unsigned char)ch)){
            if(len<MAX_CMD_LEN-1){
                if(cursor<len){
                    size_t o=len,i;
                    for(i=len;i>cursor;i--) gLineBuf[i]=gLineBuf[i-1];
                    gLineBuf[cursor]=ch; len++; cursor++; redraw(o);
                }else{
                    gLineBuf[len++]=ch; cursor=len; putChar(ch);
                }
            }else{
                putStr("\r\n!! character-overflow (128 max) start again\r\n"); errorCount[ERR_OVERFLOW]++;
                len=cursor=0; prompt();
            }
        }

    }
    return NULL;
}
