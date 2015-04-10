/* Functions for handling the bluetooth communication*/
/*Self balancing robot prepared for Freescale Event*/

/*Connection
PC						uC
TxD--------\/ ---------TxD
RxD--------/\----------RxD
*/

void uart_init (UART_MemMapPtr uartch, int sysclk, int baud)
/*Parameters:
uartch 	- channel 
sysclk 	- UART module clock frequency in kHHz
baud 	- baudrate for transmission*/


void uart_init (UART_MemMapPtr uartch, int sysclk, int baud)
{
register uint16 ubd, brfa;
uint8 temp;
/* Urochomienie zegara dla wybranego UART. */
if(uartch == UART0_BASE_PTR)
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
else if (uartch == UART1_BASE_PTR)
		SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	else if (uartch == UART2_BASE_PTR)
			SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
 
/* Upewnienie siê, ¿e nadajnik i odbiornik s¹ wy³¹czone podczas
* zmieniania ustawieñ.
*/
UART_C2_REG(uartch) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
 
/* Konfiguracja UART do trybu 8-bitowego, bez kontroli parzystoœci. */
/* Potrzeba ustawieñ pocz¹tkowych, wiêc ca³y rejestr zostaje wyczyszczony. */
UART_C1_REG(uartch) = 0;
 
/* Obliczenie ustawieñ prêdkoœci transmisji. */
ubd = (uint16)((sysclk*1000)/(baud * 16));
 
/* Zapisanie aktualnej wartoœci UARTx_BDH, oprócz bitu SBR. */
temp = UART_BDH_REG(uartch) & ~(UART_BDH_SBR(0x1F));
 
UART_BDH_REG(uartch) = temp | UART_BDH_SBR(((ubd & 0x1F00) >> 8));
UART_BDL_REG(uartch) = (uint8)(ubd & UART_BDL_SBR_MASK);
 
/* Ustalenie, czy dzielnik u³amkowy jest potrzebny w celu zbli¿enia siê do szybkoœci transmisji. */
brfa = (((sysclk*32000)/(baud * 16)) - (ubd * 32));
 
/* Zapisanie aktualnej wartoœci rejestru UARTx_C4, oprócz bitu BRFA. */
temp = UART_C4_REG(uartch) & ~(UART_C4_BRFA(0x1F));
UART_C4_REG(uartch) = temp | UART_C4_BRFA(brfa);
 
/* Uruchomienie odbiornika i nadajnika. */
UART_C2_REG(uartch) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
}