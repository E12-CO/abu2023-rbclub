void writeChipSelect(uint8_t config){
  GPIO.out_w1tc = (1<<CS_LATCH);
  for(uint8_t i = 0;i<8;i++){
    GPIO.out_w1tc = (1<<CS_CLOCK);
    GPIO.out_w1ts = ((config & 0b1 << i) && 1) << CS_DATA;
    GPIO.out_w1tc = (!((config & 0b1 << i) && 1)) << CS_DATA;
    GPIO.out_w1ts = (1<<CS_CLOCK);
  }
  GPIO.out_w1ts = (1<<CS_LATCH);
}

void startChipSelect(){
  GPIO.out_w1tc = (1<<CS_LATCH);
  GPIO.out_w1tc = (1<<CS_CLOCK);
  GPIO.out_w1tc = (1<<CS_DATA);
  GPIO.out_w1ts = (1<<CS_CLOCK);
  GPIO.out_w1ts = (1<<CS_LATCH);
}

void shiftChipSelect(){
  GPIO.out_w1tc = (1<<CS_LATCH);
  GPIO.out_w1tc = (1<<CS_CLOCK);
  GPIO.out_w1ts = (1<<CS_DATA);
  GPIO.out_w1ts = (1<<CS_CLOCK);
  GPIO.out_w1ts = (1<<CS_LATCH);
}
