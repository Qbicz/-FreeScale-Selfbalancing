/* Functions for handling the bluetooth communication*/
/*Self balancing robot prepared for Freescale Event*/

/*Connection
PC						uC
TxD--------\/ ---------TxD
RxD--------/\----------RxD
*/



/*Do zrobienia:
-funkcje inicjuj�ce piny (chapter 10 ? )
PORTC_PCR16=PORT_PCR_MUX(3);
  PORTC_PCR17=PORT_PCR_MUX(3);
  uart_init(UART3_BASE_PTR,100000000,19200);
 
  UART3_D=0xaa;


*/



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
 
/* Upewnienie si�, �e nadajnik i odbiornik s� wy��czone podczas
* zmieniania ustawie�.
*/
UART_C2_REG(uartch) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
 
/* Konfiguracja UART do trybu 8-bitowego, bez kontroli parzysto�ci. */
/* Potrzeba ustawie� pocz�tkowych, wi�c ca�y rejestr zostaje wyczyszczony. */
UART_C1_REG(uartch) = 0;
 
/* Obliczenie ustawie� pr�dko�ci transmisji. */
ubd = (uint16)((sysclk*1000)/(baud * 16));
 
/* Zapisanie aktualnej warto�ci UARTx_BDH, opr�cz bitu SBR. */
temp = UART_BDH_REG(uartch) & ~(UART_BDH_SBR(0x1F));
 
UART_BDH_REG(uartch) = temp | UART_BDH_SBR(((ubd & 0x1F00) >> 8));
UART_BDL_REG(uartch) = (uint8)(ubd & UART_BDL_SBR_MASK);
 
/* Ustalenie, czy dzielnik u�amkowy jest potrzebny w celu zbli�enia si� do szybko�ci transmisji. */
brfa = (((sysclk*32000)/(baud * 16)) - (ubd * 32));
 
/* Zapisanie aktualnej warto�ci rejestru UARTx_C4, opr�cz bitu BRFA. */
temp = UART_C4_REG(uartch) & ~(UART_C4_BRFA(0x1F));
UART_C4_REG(uartch) = temp | UART_C4_BRFA(brfa);
 
/* Uruchomienie odbiornika i nadajnika. */
UART_C2_REG(uartch) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
}

char uart_getchar (UART_MemMapPtr channel)
{
/* Oczekiwanie na wprowadzenie znaku. */
while (!(UART_S1_REG(channel) & UART_S1_RDRF_MASK));
/* Zwr�cenie 8-bitowych danych z odbiorniika. */
	return UART_D_REG(channel);
}

//sprawdzenie czy jakis znak zostal odebrany (po tej funkcji wywolamy sobie getchar
int uart_getchar_present (UART_MemMapPtr channel)
{
	return (UART_S1_REG(channel) & UART_S1_RDRF_MASK);
}

void uart_putchar (UART_MemMapPtr channel, char ch)
{
/* Oczekiwanie na udost�pnienie przestrzeni w FIFO. */
while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));
/* Wys�anie znaku. */
UART_D_REG(channel) = (uint8)ch;
}
