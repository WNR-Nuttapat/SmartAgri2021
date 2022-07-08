void handler1_channel_1(void);
void handler2_channel_1(void);
void handler3_channel_1(void);
void handler4_channel_1(void);

int RotateFRM, time4_start, time4_stop;
int RotateFLM, time1_start, time1_stop;
int RotateRRM, time3_start, time3_stop;
int RotateRLM, time2_start, time2_stop;

int dFRM=0, dFLM=0, dRRM=0, dRLM=0;
int prev_dFRM=0, prev_dFLM=0, prev_dRRM=0, prev_dRLM=0;

int FRM_rev=0,FLM_rev=0,RRM_rev=0,RLM_rev=0;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(250000);

  //RotateFRM -> TIMER4
  pinMode(PB6, INPUT);
  delay(250);
  Timer4.attachCompare1Interrupt(handler4_channel_1);
  TIMER4_BASE->CR1 = TIMER_CR1_CEN;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER4_BASE->CCMR2 = 0;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E;
  TIMER4_BASE->PSC = 17;
  TIMER4_BASE->ARR = 0xFFFF;
  TIMER4_BASE->DCR = 0;

  //RotateFLM -> TIMER1
  pinMode(PA8, INPUT);
  delay(250);
  Timer1.attachCompare1Interrupt(handler1_channel_1);
  TIMER1_BASE->CR1 = TIMER_CR1_CEN;
  TIMER1_BASE->CR2 = 0;
  TIMER1_BASE->SMCR = 0;
  TIMER1_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER1_BASE->EGR = 0;
  TIMER1_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER1_BASE->CCMR2 = 0;
  TIMER1_BASE->CCER = TIMER_CCER_CC1E;
  TIMER1_BASE->PSC = 17;
  TIMER1_BASE->ARR = 0xFFFF;
  TIMER1_BASE->DCR = 0;

  //RotateRRM -> TIMER3
  pinMode(PA6, INPUT);
  delay(250);
  Timer3.attachCompare1Interrupt(handler3_channel_1);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E;
  TIMER3_BASE->PSC = 17;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;

  //RotateRLM -> TIMER2
  pinMode(PA0, INPUT);
  delay(250);
  Timer2.attachCompare1Interrupt(handler2_channel_1);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER2_BASE->CCMR2 = 0;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E;
  TIMER2_BASE->PSC = 17;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
  dFRM = map(RotateFRM, 127, 4307, 0, 360);
  dFLM = map(RotateFLM, 127, 4307, 0, 360);
  dRRM = map(RotateRRM, 127, 4307, 0, 360);
  dRLM = map(RotateRLM, 127, 4307, 0, 360);
  
  if ((prev_dFRM >= 330 && prev_dFRM <= 360) && (dFRM >= 0 && dFRM <= 30)){
      FRM_rev++;
  }
  else if ((prev_dFRM >= 0 && prev_dFRM <= 30) && (dFRM >= 330 && dFRM <= 360)){
      FRM_rev--;
  }

  if ((prev_dFLM >= 330 && prev_dFLM <= 360) && (dFLM >= 0 && dFLM <= 30)){
      FLM_rev++;
  }
  else if ((prev_dFLM >= 0 && prev_dFLM <= 30) && (dFLM >= 330 && dFLM <= 360)){
      FLM_rev--;
  }

  if ((prev_dRRM >= 330 && prev_dRRM <= 360) && (dRRM >= 0 && dRRM <= 30)){
      RRM_rev++;
  }
  else if ((prev_dRRM >= 0 && prev_dRRM <= 30) && (dRRM >= 330 && dRRM <= 360)){
      RRM_rev--;
  }

  if ((prev_dRLM >= 330 && prev_dRLM <= 360) && (dRLM >= 0 && dRLM <= 30)){
      RLM_rev++;
  }
  else if ((prev_dRLM >= 0 && prev_dRLM <= 30) && (dRLM >= 330 && dRLM <= 360)){
      RLM_rev--;
  }

  prev_dFRM = dFRM; prev_dFLM = dFLM;
  prev_dRRM = dRRM; prev_dRLM = dRLM;

  Serial1.print('E'); Serial1.print(FRM_rev); Serial1.print(','); 
  Serial1.print('F'); Serial1.print(FLM_rev); Serial1.print(','); 
  Serial1.print('G'); Serial1.print(RRM_rev); Serial1.print(','); 
  Serial1.print('H'); Serial1.print(RLM_rev); Serial1.println("");

  
}

void handler4_channel_1(void) {
  if (TIMER4_BASE->SR & 1 << 1) {
    TIMER4_BASE->SR &= ~(1 << 1);
  }
  if ((1 << 6) & GPIOB_BASE->IDR) {
    time4_start = TIMER4_BASE->CCR1;
    TIMER4_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    time4_stop = TIMER4_BASE->CCR1;
    RotateFRM = time4_stop - time4_start;
    if (RotateFRM < 0)
      RotateFRM += 0xFFFF;
    TIMER4_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler1_channel_1(void) {
  if (TIMER1_BASE->SR & 1 << 1) {
    TIMER1_BASE->SR &= ~(1 << 1);
  }
  if ((1 << 8) & GPIOA_BASE->IDR) {
    time1_start = TIMER1_BASE->CCR1;
    TIMER1_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    time1_stop = TIMER1_BASE->CCR1;
    RotateFLM = time1_stop - time1_start;
    if (RotateFLM < 0)
      RotateFLM += 0xFFFF;
    TIMER1_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler3_channel_1(void) {
  if (TIMER3_BASE->SR & 1 << 1) {
    TIMER3_BASE->SR &= ~(1 << 1);
  }

  if ((1 << 6) & GPIOA_BASE->IDR) {
    time3_start = TIMER3_BASE->CCR1;
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    time3_stop = TIMER3_BASE->CCR1;
    RotateRRM = time3_stop - time3_start;
    if (RotateRRM < 0)
      RotateRRM += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler2_channel_1(void) {
  if (TIMER2_BASE->SR & 1 << 1) {
    TIMER2_BASE->SR &= ~(1 << 1);
  }
  if ((1 << 0) & GPIOA_BASE->IDR) {
    time2_start = TIMER2_BASE->CCR1;
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    time2_stop = TIMER2_BASE->CCR1;
    RotateRLM = time2_stop - time2_start;
    if (RotateRLM < 0)
      RotateRLM += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}
