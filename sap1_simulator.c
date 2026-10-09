#include <stdio.h>
#include <stdbool.h>

// ============================================================================
// SIMULATOR SAP-1 V3: CONTROLLER-SEQUENCER + BUS YANG REALISTIS
// ============================================================================
// Format CON (12 Bit):
// Bit:  11   10    9    8    7    6    5    4    3    2    1    0
// Name: Cp   Ep  ~Lm  ~CE  ~Li  ~Ei  ~La   Ea   Su   Eu  ~Lb  ~Lo
// Type: AH   AH   AL   AL   AL   AL   AL   AH   AH   AH   AL   AL
// ============================================================================

//mendefinisikan posisi bit untuk setiap sinyal kontrol
#define BIT_CP  (1 << 11) // berada di bit ke-11
#define BIT_EP  (1 << 10)
#define BIT_LM  (1 << 9)
#define BIT_CE  (1 << 8)
#define BIT_LI  (1 << 7)
#define BIT_EI  (1 << 6)
#define BIT_LA  (1 << 5)
#define BIT_EA  (1 << 4)
#define BIT_SU  (1 << 3)
#define BIT_EU  (1 << 2)
#define BIT_LB  (1 << 1)
#define BIT_LO  (1 << 0)

#define CON_INACTIVE 0x3E3 //keadaan default semua sinyal kontrol dalam kondisi tidak aktif. AH dibuat 0, AL dibuat 1.
// "Menyalakan" sinyal = membalik bitnya dari kondisi inaktif.
// AH: 0 -> 1, AL: 1 -> 0. 

#define ASSERT(mask) ((unsigned short)(CON_INACTIVE ^ (mask)))
//mengaktifkan sinyal tertentu dengan cara membalik bit dari kondisi default memakai XOR

#define MAX_CLOCKS 600  //membatasi jumlah clock agar program tidak berjalan tanpa akhir jika tidak pernah mencapai `HLT`

typedef enum { // mendefinisikan jenis eror yang mungkin terjadi
    FAULT_NONE = 0,
    FAULT_BUS_CONTENTION,
    FAULT_FLOATING_BUS,
    FAULT_ILLEGAL_OPCODE,
    FAULT_WATCHDOG
} Fault;

unsigned char PC = 0, MAR = 0, RAM[16], IR = 0, A = 0, B = 0, OUT_REG = 0;

unsigned char W_BUS = 0;
bool          BUS_VALID = false;        // false = bus mengambang (tidak ada driver)
const char*   BUS_DRIVER = "-";

unsigned short CON = CON_INACTIVE;
bool HLT_FLAG = false;
Fault FAULT = FAULT_NONE;
int  T_STATE = 1;                       // ring counter 1..6
int  CLOCK_COUNT = 0;

const char* fault_text(Fault f) {
    switch (f) {
        case FAULT_BUS_CONTENTION: return "BUS CONTENTION (lebih dari satu driver aktif)";
        case FAULT_FLOATING_BUS:   return "FLOATING BUS (ada penerima aktif tetapi tidak ada driver)";
        case FAULT_ILLEGAL_OPCODE: return "OPCODE ILEGAL";
        case FAULT_WATCHDOG:       return "WATCHDOG (melebihi batas clock, kemungkinan tanpa HLT)";
        default:                   return "tidak ada";
    }
}

void print_binary12(unsigned short v) {
    for (int i = 11; i >= 0; i--) { printf("%d", (v >> i) & 1); if (i == 8 || i == 4) printf(" "); }
}
void print_binary4(unsigned char v) { for (int i = 3; i >= 0; i--) printf("%d", (v >> i) & 1); } // cocok untuk PC, MAR
void print_binary8(unsigned char v) { for (int i = 7; i >= 0; i--) printf("%d", (v >> i) & 1); } // cocok untuk IR, A, B, OUT_REG

void print_hardware_state(int t, const char* label) {
    printf("[T%d] CON: ", t);
    print_binary12(CON);
    printf(" (0x%03X) | ", CON);
    if (BUS_VALID) {
        printf("W:0x%02X(%s)", W_BUS, BUS_DRIVER);
    } else {
        printf("W: --(float)");
    }
    printf(" | PC:");  print_binary4(PC);
    printf(" MAR:");   print_binary4(MAR);
    printf(" | IR:0x%02X A:%3d B:%3d OUT:%3d <-- %s\n", IR, A, B, OUT_REG, label);
}

void execute_control_word(void) {
    // ---- TAHAP 1: kumpulkan SEMUA driver, tidak berhenti di yang pertama ----
    int drivers = 0;
    unsigned char value = 0;
    const char* name = "-";

    if (CON & BIT_EP)          { drivers++; value = PC & 0x0F;  name = "PC"; }
    if ((CON & BIT_EI) == 0)   { drivers++; value = IR & 0x0F;  name = "IR"; }
    if ((CON & BIT_CE) == 0)   { drivers++; value = RAM[MAR];   name = "RAM"; }
    if (CON & BIT_EA)          { drivers++; value = A;          name = "A"; }
    if (CON & BIT_EU){ 
        drivers++; name = "ALU";
        if (CON & BIT_SU) value = (A - B) & 0xFF;   // Su=1: pengurangan
        else              value = (A + B) & 0xFF;   // Su=0: penjumlahan
    }

    if (drivers > 1) { FAULT = FAULT_BUS_CONTENTION; BUS_VALID = false; BUS_DRIVER = "CONFLICT"; return; }
    if (drivers == 1) { W_BUS = value; BUS_VALID = true; BUS_DRIVER = name; }
    else              { W_BUS = 0;     BUS_VALID = false; BUS_DRIVER = "-"; }

    // ---- TAHAP 2: penerima. Tanpa driver, penerima tidak boleh memuat ----
    bool any_receiver = ((CON & BIT_LM) == 0) || ((CON & BIT_LI) == 0) ||
                        ((CON & BIT_LA) == 0) || ((CON & BIT_LB) == 0) ||
                        ((CON & BIT_LO) == 0);
    if (any_receiver && !BUS_VALID) { FAULT = FAULT_FLOATING_BUS; return; }

    if ((CON & BIT_LM) == 0) MAR = W_BUS & 0x0F;
    if ((CON & BIT_LI) == 0) IR  = W_BUS;
    if ((CON & BIT_LA) == 0) A   = W_BUS;
    if ((CON & BIT_LB) == 0) B   = W_BUS;
    if ((CON & BIT_LO) == 0) OUT_REG = W_BUS;

    // ---- TAHAP 3: counter ----
    if (CON & BIT_CP) PC = (PC + 1) & 0x0F;
}

void load_program(void) {
    for (int i = 0; i < 16; i++) RAM[i] = 0;
    RAM[0x0] = 0x09;  // LDA 9H (16)
    RAM[0x1] = 0x1A;  // ADD AH (20)
    RAM[0x2] = 0x1B;  // ADD BH (24)
    RAM[0x3] = 0x2C;  // SUB CH (32)
    RAM[0x4] = 0xE0;  // OUT
    RAM[0x5] = 0xF0;  // HLT
    RAM[0x9] = 16; RAM[0xA] = 20; RAM[0xB] = 24; RAM[0xC] = 32;
}

typedef struct { unsigned short con; const char* label; 
    #define OP_LDA 0x0
    #define OP_ADD 0x1
    #define OP_SUB 0x2
    #define OP_OUT 0xE
    #define OP_HLT 0xF
} MicroOp;

static MicroOp FETCH_ROM[3];
static MicroOp EXEC_ROM[16][3];     // T4..T6
static bool    OPCODE_DEFINED[16];

void init_control_rom(void) {
    // Fetch: sama untuk semua instruksi
    FETCH_ROM[0] = (MicroOp){ ASSERT(BIT_EP | BIT_LM), "Ep=1, ~Lm=0 (PC -> MAR)" }; // address state
    FETCH_ROM[1] = (MicroOp){ ASSERT(BIT_CP),          "Cp=1 (PC = PC + 1)" }; // increment state
    FETCH_ROM[2] = (MicroOp){ ASSERT(BIT_CE | BIT_LI), "~CE=0, ~Li=0 (RAM -> IR)" }; //memory state

    // Default semua NOP
    for (int op = 0; op < 16; op++)
        for (int t = 0; t < 3; t++)
            EXEC_ROM[op][t] = (MicroOp){ ASSERT(0), "NOP" };

    MicroOp t4_addr = { ASSERT(BIT_EI | BIT_LM), "~Ei=0, ~Lm=0 (IR[3:0] -> MAR)" };

    // LDA
    EXEC_ROM[OP_LDA][0] = t4_addr;
    EXEC_ROM[OP_LDA][1] = (MicroOp){ ASSERT(BIT_CE | BIT_LA), "~CE=0, ~La=0 (RAM -> Accumulator)" };
    OPCODE_DEFINED[OP_LDA] = true;

    // ADD
    EXEC_ROM[OP_ADD][0] = t4_addr;
    EXEC_ROM[OP_ADD][1] = (MicroOp){ ASSERT(BIT_CE | BIT_LB), "~CE=0, ~Lb=0 (RAM -> Reg B)" };
    EXEC_ROM[OP_ADD][2] = (MicroOp){ ASSERT(BIT_EU | BIT_LA), "Eu=1, ~La=0 (ALU Add -> Accumulator)" };
    OPCODE_DEFINED[OP_ADD] = true;

    // SUB
    EXEC_ROM[OP_SUB][0] = t4_addr;
    EXEC_ROM[OP_SUB][1] = (MicroOp){ ASSERT(BIT_CE | BIT_LB), "~CE=0, ~Lb=0 (RAM -> Reg B)" };
    EXEC_ROM[OP_SUB][2] = (MicroOp){ ASSERT(BIT_SU | BIT_EU | BIT_LA), "Su=1, Eu=1, ~La=0 (ALU Sub -> Accumulator)" };
    OPCODE_DEFINED[OP_SUB] = true;

    // OUT
    EXEC_ROM[OP_OUT][0] = (MicroOp){ ASSERT(BIT_EA | BIT_LO), "Ea=1, ~Lo=0 (Accumulator -> Output Reg)" };
    OPCODE_DEFINED[OP_OUT] = true;

    // HLT: menghentikan proses ekseskusi. Pada T4, tidak ada sinyal kontrol yang aktif.
    EXEC_ROM[OP_HLT][0] = (MicroOp){ ASSERT(0), "HLT (Clock Generator Stopped)" };
    OPCODE_DEFINED[OP_HLT] = true;
}

// Satu pulsa clock = satu T-state
void clock_pulse(void) {
    if (++CLOCK_COUNT > MAX_CLOCKS) { FAULT = FAULT_WATCHDOG; return; }

    MicroOp mop;
    if (T_STATE <= 3) {
        mop = FETCH_ROM[T_STATE - 1];
    } else {
        unsigned char opcode = (IR >> 4) & 0x0F;
        if (!OPCODE_DEFINED[opcode]) { FAULT = FAULT_ILLEGAL_OPCODE; return; }
        mop = EXEC_ROM[opcode][T_STATE - 4];
        if (opcode == OP_HLT && T_STATE == 4) HLT_FLAG = true;
    }

    if (T_STATE == 1)
        printf("\n==== MACHINE CYCLE BARU (PC = %d) | CON: Cp Ep ~Lm ~CE . ~Li ~Ei ~La Ea . Su Eu ~Lb ~Lo ====\n", PC);
    CON = mop.con;
    execute_control_word();
    if (FAULT != FAULT_NONE) return;
    print_hardware_state(T_STATE, mop.label);

    // Ring counter: T1..T6 lalu kembali ke T1. Berhenti jika HLT.
    T_STATE = (T_STATE % 6) + 1;
}

int main(void) {
    printf("============================================================================\n");
    printf("   SIMULATOR HARDWARE SAP-1 V3: CONTROL ROM + RING COUNTER + BUS ARBITER\n");
    printf("============================================================================\n");

    init_control_rom(); //mengisi ROM kontrol dengan sinyal kontrol untuk setiap instruksi
    load_program(); //memuat program dan data ke RAM

    while (!HLT_FLAG && FAULT == FAULT_NONE) clock_pulse();

    printf("\n============================================================================\n");
    if (FAULT != FAULT_NONE) {
        printf("EKSEKUSI DIHENTIKAN OLEH FAULT: %s\n", fault_text(FAULT));
        printf("T-state: T%d | PC=%d | Clock ke-%d | CON=", T_STATE, PC, CLOCK_COUNT);
        print_binary12(CON); printf("\n");
        printf("============================================================================\n");
        return 1;
    }
    printf("EKSEKUSI SELESAI (%d clock)\n", CLOCK_COUNT);
    printf("Tampilan LED Output Register (Binary): "); print_binary8(OUT_REG);
    printf("\nHasil Akhir Desimal (unsigned) : %d\n", OUT_REG);
    printf("Hasil Akhir Desimal (bertanda)  : %d\n", (signed char)OUT_REG);
    printf("Hasil Akhir Hex                 : 0x%02X\n", OUT_REG);
    printf("============================================================================\n");
    return 0;
}