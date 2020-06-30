#include "mbed.h"
#include "bbcar.h"
#include "mbed_rpc.h"

//motor start
#define CENTER_BASE 1500
PwmOut servo_LEFT(D9);
PwmOut servo_RIGHT(D8);
//motot end


//encoder start 
DigitalIn encoder(D3);
Ticker encoder_ticker;
volatile int steps;
int dick = 0;
volatile int last;
//encoder end

// ping
DigitalInOut  ping_origin(D10);
parallax_ping  ping(ping_origin);
//DigitalOut stop(LED3);

RawSerial xbee(D12, D11);
Serial uart(D1, D0);
Serial pc(USBTX, USBRX);

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;


void servo_control_LEFT(int speed){
    if (speed > 200)       speed = 200;
    else if (speed < -200) speed = -200;
    servo_LEFT = (CENTER_BASE + speed)/20000.0f;
}


void servo_control_RIGHT(int speed){
    if (speed > 200)       speed = 200;
    else if (speed < -200) speed = -200;
    servo_RIGHT = (CENTER_BASE + speed)/20000.0f;
}


void encoder_control(){
    int value = encoder;

    if(!last && value){
        steps++;
        load++;
        }
    last = value;
}

void miss_2(){
    float ping_dis[3];
    for(int i = 0; i < 4; i ++){
      ping_dis[i] = float(ping1);
    }
    if(ping_dis[0] > ping_dis[1] && ping_dis[2] < ping_dis[3]) { //triangle
      xbee.printf("Triangle!!\r\n");
    }else if(ping_dis[0] == ping_dis[1] && ping_dis[1] == ping_dis[2] && ping_dis[2] == ping_dis[3]) { //rectangle
      xbee.printf("Rectangle!!\r\n");
    }else if(ping_dis[0] > ping_dis[1] && ping_dis[1] > ping_dis[2] && ping_dis[2] > ping_dis[3]) { //right triangle
      xbee.printf("Right Triangle!!\r\n");
    }else {
      xbee.printf("Tooth Pattern!!\r\n");
    }
}

void Logfile(Arguments *in, Reply *out);
RPCFunction rpcLog(&getLog, "getLog");

//XBee
void xbee_setting();
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);


int stop = 0;

int main() {

    pc.baud(9600);
    xbee.baud(9600);
    uart.baud(9600);
    xbee_setting();

    servo_LEFT.period(.02);
    servo_RIGHT.period(.02);

    encoder_ticker.attach(&encoder_control, .001);

    steps = 0;
    last = 0;


    
    while(1) {
        if(load <220){
            servo_control_LEFT(99);
            servo_control_RIGHT(-84); //right wheel is special.
        }//go straight

        else if(load >=220 && load < 232){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                //wait(0.5);
                stop = 1;
            }
            servo_control_LEFT(-70);
            servo_control_RIGHT(-70); //right wheel is special.            
        } //turn left

        else if(load >= 232 && load <390){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                //wait(0.5);
                stop = 0;                
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-87);            
        }//go straight to mission 1
   
        else if(load >=390 && load < 403){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(1);
                stop = 1;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //turn right

        else if(load >=403 && load < 450){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(-94);
            servo_control_RIGHT(89); //right wheel is special.            
        } //go backward park        

        else if(load >=450 && load < 480){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(1);
                stop = 1;
            }
            servo_control_LEFT(85);
            servo_control_RIGHT(-91); //right wheel is special.            
        } //go straight after park

        else if(load >=480 && load < 493){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(1);
                stop = 0;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //turn right

        else if(load >=493 && load < 580){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-87); //right wheel is special.            
        } //go straight 

        else if(load >=580 && load < 592){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(1);
                stop = 0;
            }
            servo_control_LEFT(-70);
            servo_control_RIGHT(-70); //right wheel is special.            
        } //take pic(left)

        else if(load >=592 && load < 600){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(1);
                stop = 1;
            }
            servo_control_LEFT(-94);
            servo_control_RIGHT(85); //right wheel is special.            
        } //go backward

        else if(load >=600 && load < 618){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(5);
                stop = 0;
            }
            if(uart.readable()){
                char recv = uart.getc();
                xbee.putc(recv);
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-85); //right wheel is special.            
        } //go forward

        else if(load >=618 && load < 630){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(1);
                stop = 1;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //take pic(right)

        else if(load >=630 && load < 650){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(85);
            servo_control_RIGHT(-91); //right wheel is special.            
        } //go straight to miossion2

        else if(load >=650 && load < 662){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //turn right

        else if(load >=662 && load < 840){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-88); //right wheel is special.            
        } //go straight to miossion2

        else if(load >=840 && load < 853){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //turn right

        else if(load >=853 && load < 950){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-88); //right wheel is special.            
        } //go straight to miossion2

        else if(load >=950 && load < 963){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //turn right

        else if(load >=963 && load < 980){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-88); //right wheel is special.            
        } //go straight to miossion2        

        else if(load >=980 && load < 993){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            servo_control_LEFT(-70);
            servo_control_RIGHT(-70); //right wheel is special.            
        } //turn left

        else if(load >=993 && load < 1000){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(0); //right wheel is special.            
        } //turn right

        else if(load >=1000 && load < 1010){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            miss_2();
            servo_control_LEFT(-70);
            servo_control_RIGHT(0); //right wheel is special.            
        } //turn left

        else if(load >=1010 && load < 1020){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(-70);
            servo_control_RIGHT(-70); //right wheel is special.            
        } //turn left

        else if(load >=1020 && load < 1033){
            if(stop==0){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 1;
            }
            servo_control_LEFT(70);
            servo_control_RIGHT(70); //right wheel is special.            
        } //turn right

        else if(load >=1033 && load < 1500){
            if(stop==1){
                servo_control_LEFT(0);
                servo_control_RIGHT(0);
                wait(2);
                stop = 0;
            }
            servo_control_LEFT(94);
            servo_control_RIGHT(-88); //right wheel is special.            
        } //go straight to miossion2  


        else{
            servo_control_LEFT(0);
            servo_control_RIGHT(0);
        }

        wait(0.01);
    }
    
}

void Logfile(Arguments *in, Reply *out){
    if(dick <220){
        xbee.printf("go forward to start mission1\r\n");
    }//go straight
    else if(dick >=220 && dick < 232){
        xbee.printf("turn left\r\n");         
    } //turn left
    else if(dick >= 232 && dick <390){
        xbee.printf("go straight to mission 1\r\n");            
    }//go straight to mission 1
    else if(dick >=390 && dick < 403){
        xbee.printf("turn right\r\n");          
    } //turn right
    else if(dick >=403 && dick < 450){
        xbee.printf("go backward park\r\n");          
    } //go backward park        
    else if(dick >=450 && dick < 480){
        xbee.printf("go straight after park\r\n");           
    } //go straight after park
    else if(dick >=480 && dick < 493){
        xbee.printf("turn right\r\n");            
    } //turn right
    else if(dick >=493 && dick < 580){
        xbee.printf("go straight\r\n");            
    } //go straight 
    else if(dick >=580 && dick < 592){
        xbee.printf("take pic\r\n");           
    } //take pic(left)
    else if(dick >=592 && dick < 600){
        xbee.printf("go backward\r\n");       
    } //go backward
    else if(dick >=600 && dick < 618){
        xbee.printf("go straight\r\n");             
    } //go forward
    else if(dick >=618 && dick < 630){
        xbee.printf("take pic\r\n");          
    } //take pic(right)
    else if(dick >=630 && dick < 650){
        xbee.printf("go straight\r\n");            
    } //go straight to miossion2
    else if(dick >=650 && dick < 662){
        xbee.printf("turn right\r\n");          
    } //turn right
    else if(dick >=662 && dick < 840){
        xbee.printf("go straight\r\n");            
    } //go straight to miossion2
    else if(dick >=840 && dick < 853){
        xbee.printf("turn right\r\n");           
    } //turn right
    else if(dick >=853 && dick < 950){
        xbee.printf("go straight\r\n");          
    } //go straight to miossion2
    else if(dick >=950 && dick < 963){
        xbee.printf("turn right\r\n");           
    } //turn right
    else if(dick >=963 && dick < 980){
         xbee.printf("go straight\r\n");           
    } //go straight to miossion2        
    else if(dick >=980 && dick < 993){
        xbee.printf("turn left\r\n");           
    } //turn left
    else if(dick >=993 && dick < 1000){
        xbee.printf("turn right\r\n");         
    } //turn right
    else if(dick >=1000 && dick < 1010){
        xbee.printf("turn left\r\n");         
    } //turn left
    else if(dick >=1010 && dick < 1020){
        xbee.printf("turn left\r\n");           
    } //turn left
    else if(dick >=1020 && dick < 1033){
        xbee.printf("turn right\r\n");       
    } //turn right
    else if(dick >=1033 && dick < 1500){
        xbee.printf("end\r\n");          
    } //go straight to miossion2
    else{
        xbee.printf("no action\r\n");
    }
}

void xbee_setting(){
  // XBee setting
  char xbee_reply[4];
  xbee.baud(9600);
  xbee.printf("+++");
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
    pc.printf("enter AT mode.\r\n");
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
  }

  xbee.printf("ATMY 0x223\r\n");
  reply_messange(xbee_reply, "setting MY : 0x223");

  xbee.printf("ATDL 0x123\r\n");
  reply_messange(xbee_reply, "setting DL : 0x123");

  xbee.printf("ATID 0x1\r\n");
  reply_messange(xbee_reply, "setting PAN ID : 0x1");

  xbee.printf("ATWR\r\n");
  reply_messange(xbee_reply, "write config");

  xbee.printf("ATMY\r\n");
  check_addr(xbee_reply, "MY");

  xbee.printf("ATDL\r\n");
  check_addr(xbee_reply, "DL");

  xbee.printf("ATCN\r\n");
  reply_messange(xbee_reply, "exit AT mode");
  xbee.getc();

  // start
  pc.printf("start\r\n");
  t.start(callback(&queue, &EventQueue::dispatch_forever));

  // Setup a serial interrupt function to receive data from xbee
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
}


void xbee_rx_interrupt(void)
{

  xbee.attach(NULL, Serial::RxIrq); // detach interrupt

  queue.call(&xbee_rx);
}


void xbee_rx(void)
{
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      if (recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    RPC::call(buf, outbuf);
    pc.printf("%s\r\n", outbuf);
    wait(0.1);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}


void reply_messange(char *xbee_reply, char *messange){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){

   pc.printf("%s\r\n", messange);

   xbee_reply[0] = '\0';

   xbee_reply[1] = '\0';

   xbee_reply[2] = '\0';

  }

}


void check_addr(char *xbee_reply, char *messenger){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  xbee_reply[3] = xbee.getc();

  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

  xbee_reply[0] = '\0';

  xbee_reply[1] = '\0';

  xbee_reply[2] = '\0';

  xbee_reply[3] = '\0';

}
