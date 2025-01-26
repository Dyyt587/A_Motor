#include "sys.h"
void Init_System_Parameter(void);
void Init_Motor_Parameter(void);
void Init_Control_Parameter(void);
void Exchange_motor_code(void);
void Process_Store_parameter(void);
u8 MemReadHalfWord(u16 *data,u16 vaddr,u16 num) ;
u8 MemWriteHalfWord(u16 *data,u16 vaddr,u16 num) ;
u8 MemReadModbus(u16 modbus_addr,u16 vaddr,u16 num) ;
u8 MemWriteModbus(u16 modbus_addr,u16 vaddr,u16 num);
